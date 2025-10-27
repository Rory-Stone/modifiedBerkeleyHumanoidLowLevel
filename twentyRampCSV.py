# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, math, struct, os, csv
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ---------------- user knobs ----------------
SWEEP_MIN_W     = -20.0     # [rad/s]
SWEEP_MAX_W     = +20.0     # [rad/s]
SWEEP_STEP_W    = 2.0       # [rad/s] step between plateaus (use 1.0 for finer)
RAMP_TO_FIRST_S = 1.2       # [s] smooth ramp 0 -> +20 so it doesn't stall
SETTLE_S        = 0.25      # [s] let speed settle at each step (shorter = faster)
LOG_S           = 0.45      # [s] record window per step (shorter = faster)
LOOP_HZ         = 40.0      # main loop rate (alternates vel/torque PDO)
VEL_KP          = 0.20
VEL_KI          = 0.005
TORQUE_LIMIT_NM = 0.20
TIMEOUT_MS      = 8         # CAN RX timeout (if supported)
PRINT_DECIMATE  = 4         # print every N ticks (lower = chattier)
DO_RETURN_SWEEP = True      # do both directions (20->-20->20)
# --------------------------------------------

def read_torque_pdo3(bus, device_id, timeout_s=0.008):
    """
    Fast torque read via PDO-3:
      TX: RECEIVE_PDO_3 with [position_target, torque_target] placeholders
      RX: TRANSMIT_PDO_3 with [position_measured, torque_measured]
    """
    bus.transmit(recoil.CANFrame(
        device_id,
        recoil.Function.RECEIVE_PDO_3,
        size=8,
        data=struct.pack("<ff", 0.0, 0.0)
    ))
    rx = bus.receive(
        filter_device_id=device_id,
        filter_function=recoil.Function.TRANSMIT_PDO_3,
        timeout=timeout_s
    )
    if not rx:
        return None, None
    pos_meas, tau_meas = struct.unpack("<ff", rx.data[0:8])
    return float(pos_meas), float(tau_meas)

def ramp_to(bus, dev, w_start, w_target, ramp_time_s, loop_hz, timeout_s):
    """Linear ramp from w_start -> w_target over ramp_time_s."""
    if ramp_time_s <= 0:
        return w_target
    steps = max(1, int(round(ramp_time_s * loop_hz)))
    for i in range(1, steps + 1):
        w_cmd = w_start + (w_target - w_start) * (i / steps)
        # cheap velocity read + command in one PDO-2
        _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
        bus.feed(dev)
        time.sleep(1.0 / loop_hz)
    return w_target

def hold_and_log(bus, dev, w_target, settle_s, log_s, loop_hz, timeout_s,
                 rows, tick_state):
    """
    Hold at w_target for settle_s + log_s. During log window,
    append (t, speed_meas, torque, w_cmd) to rows (buffered).
    tick_state holds [tick, last_vel, last_tau, t0] and is updated in-place.
    """
    n_settle = max(0, int(round(settle_s * loop_hz)))
    n_log    = max(1, int(round(log_s * loop_hz)))
    tick, last_vel, last_tau, t0 = tick_state

    # settle
    for _ in range(n_settle):
        if (tick % 2) == 0:
            _p, vel = bus.write_read_pdo_2(dev, 0.0, float(w_target))
            if vel is not None and math.isfinite(vel): last_vel = float(vel)
        else:
            _p, tau = read_torque_pdo3(bus, dev, timeout_s=timeout_s)
            if tau is not None and math.isfinite(tau): last_tau = float(tau)
        bus.feed(dev)

        if PRINT_DECIMATE == 1 or (tick % PRINT_DECIMATE == 0):
            if last_vel is not None:
                txt_t = "NA" if last_tau is None else f"{last_tau:+.4f} N·m"
                print(f"Hold (settle)  w_cmd:{w_target:+.2f}  vel:{last_vel:+.3f}  tau:{txt_t}")
        tick += 1
        time.sleep(1.0 / loop_hz)

    # log
    for _ in range(n_log):
        now = time.time()
        if (tick % 2) == 0:
            _p, vel = bus.write_read_pdo_2(dev, 0.0, float(w_target))
            if vel is not None and math.isfinite(vel): last_vel = float(vel)
        else:
            _p, tau = read_torque_pdo3(bus, dev, timeout_s=timeout_s)
            if tau is not None and math.isfinite(tau): last_tau = float(tau)
            # log at torque tick when we have a recent velocity
            if last_vel is not None and last_tau is not None:
                rows.append((now - t0, last_vel, last_tau, float(w_target)))
        bus.feed(dev)

        if PRINT_DECIMATE == 1 or (tick % PRINT_DECIMATE == 0):
            if last_vel is not None:
                txt_t = "NA" if last_tau is None else f"{last_tau:+.4f} N·m"
                print(f"Log           w_cmd:{w_target:+.2f}  vel:{last_vel:+.3f}  tau:{txt_t}")
        tick += 1
        time.sleep(1.0 / loop_hz)

    tick_state[0] = tick
    tick_state[1] = last_vel
    tick_state[2] = last_tau
    return

def main():
    args = recoil.util.get_args()   # e.g., -c can0 -i 1
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # Slightly longer RX timeout helps in VMs (if available).
    if hasattr(bus, "set_timeout_ms"):
        try: bus.set_timeout_ms(int(TIMEOUT_MS))
        except Exception: pass

    # Velocity loop + safety
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    # build sweep order
    # Start with a spin-up ramp to +SWEEP_MAX_W so it won't stall.
    # Then sweep through steps (both directions if enabled).
    if SWEEP_STEP_W <= 0:
        raise ValueError("SWEEP_STEP_W must be > 0")

    down = [SWEEP_MAX_W - i*SWEEP_STEP_W
            for i in range(int(round((SWEEP_MAX_W - SWEEP_MIN_W)/SWEEP_STEP_W)) + 1)]
    down = [round(x, 6) for x in down]  # tidy floats
    up   = [SWEEP_MIN_W + i*SWEEP_STEP_W
            for i in range(int(round((SWEEP_MAX_W - SWEEP_MIN_W)/SWEEP_STEP_W)) + 1)]
    up   = [round(x, 6) for x in up]

    sweep_seq = down + (up if DO_RETURN_SWEEP else [])
    # remove duplicate between concatenations
    dedup = []
    for w in sweep_seq:
        if not dedup or abs(dedup[-1] - w) > 1e-9:
            dedup.append(w)
    sweep_seq = dedup

    rows = []  # (time_s, speed_meas_rad_per_s, torque_Nm, cmd_speed_rad_per_s)
    t0 = time.time()
    tick_state = [0, None, None, t0]  # [tick, last_vel, last_tau, t0]

    rate = RateLimiter(frequency=LOOP_HZ)

    try:
        print(f"Spin-up ramp to +{SWEEP_MAX_W:.1f} rad/s over {RAMP_TO_FIRST_S:.2f}s …")
        _ = ramp_to(bus, dev, 0.0, SWEEP_MAX_W, RAMP_TO_FIRST_S, LOOP_HZ, TIMEOUT_MS/1000.0)

        print("Starting stepped sweep …")
        for w_target in sweep_seq:
            hold_and_log(bus, dev, w_target,
                         settle_s=SETTLE_S,
                         log_s=LOG_S,
                         loop_hz=LOOP_HZ,
                         timeout_s=TIMEOUT_MS/1000.0,
                         rows=rows,
                         tick_state=tick_state)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Write CSV once at end (no disk I/O in-loop)
        os.makedirs("data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = f"data/vel_torque_sweep_fastpdo_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "speed_meas_rad_per_s", "torque_Nm", "cmd_speed_rad_per_s"])
            for t_s, v_meas, tau, w_cmd in rows:
                w.writerow([f"{t_s:.3f}", f"{v_meas:.6f}", f"{tau:.6f}", f"{w_cmd:.6f}"])

        try:
            bus.set_mode(dev, recoil.Mode.IDLE)
        except Exception:
            pass
        try:
            bus.stop()
        except Exception:
            pass
        print(f"CSV saved: {path}")

if __name__ == "__main__":
    main()
