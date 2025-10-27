# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, os, csv, math, struct
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ------------- user knobs -------------
START_W = -10.0          # start speed [rad/s]
END_W   = +10.0          # end speed   [rad/s]
PRERAMP_S   = 3.0        # seconds: 0 -> START_W (gentle)
RAMP_TIME_S = 120.0      # seconds: START_W -> END_W (slow sweep)

LOOP_HZ = 40.0           # main loop rate
CTRL_TICKS_PER_TORQUE = 1  # pattern: 1 control tick, then 1 torque tick (alternate)
INTERFRAME_GAP_S = 0.0   # not needed for PDO-3; leave 0.0 (only for SDO fallback)

VEL_KP = 0.2
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20

PRINT_DECIMATE = 1       # print every N ticks (1 = every tick)
TIMEOUT_MS = 8           # CAN RX timeout if supported
# --------------------------------------

def read_torque_pdo3(bus, device_id, timeout_s=0.008):
    """
    Fast torque read via PDO-3:
      TX: RECEIVE_PDO_3  [position_target, torque_target] placeholders
      RX: TRANSMIT_PDO_3 [position_measured, torque_measured]
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

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # Slightly longer timeout helps in VMs (if available)
    if hasattr(bus, "set_timeout_ms"):
        try: bus.set_timeout_ms(int(TIMEOUT_MS))
        except Exception: pass

    # Controller setup + safety
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    # Buffer samples; write CSV after the sweep (no disk I/O in loop)
    rows = []  # (time_s, speed_cmd_rad_per_s, speed_meas_rad_per_s, torque_Nm)

    t_start = time.time()
    last_vel = None
    last_tau = None

    pattern_len = CTRL_TICKS_PER_TORQUE + 1  # e.g., 1 control tick + 1 torque tick
    tick = 0

    def sweep(duration_s: float, start_w: float, end_w: float):
        nonlocal tick, last_vel, last_tau
        t0 = time.time()
        span = end_w - start_w
        while True:
            now = time.time()
            t = now - t0
            frac = 1.0 if duration_s <= 0 else max(0.0, min(1.0, t / duration_s))
            w_cmd = start_w + span * frac

            slot = tick % pattern_len
            if slot < CTRL_TICKS_PER_TORQUE:
                # CONTROL TICK: fast PDO-2 (command vel & read pos/vel)
                _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
                bus.feed(dev)
                if vel is not None and math.isfinite(vel):
                    last_vel = float(vel)
            else:
                # TORQUE TICK: fast PDO-3 (read torque only)
                _p, tau = read_torque_pdo3(bus, dev, timeout_s=TIMEOUT_MS/1000.0)
                bus.feed(dev)
                if tau is not None and math.isfinite(tau):
                    last_tau = float(tau)
                    # Log only when we got a fresh torque AND we have a recent velocity
                    if last_vel is not None and math.isfinite(last_vel):
                        rows.append((now - t_start, float(w_cmd), last_vel, last_tau))

            # Console print for monitoring (MoveActuator style)
            if (PRINT_DECIMATE == 1) or (tick % PRINT_DECIMATE == 0):
                if last_vel is not None and math.isfinite(last_vel):
                    if last_tau is None or not math.isfinite(last_tau):
                        print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: NA")
                    else:
                        print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: {last_tau:+.4f} N·m")

            if frac >= 1.0:
                break
            tick += 1
            rate.sleep()

    try:
        print(f"Sweep: {START_W:+.1f} → {END_W:+.1f} rad/s using fast PDO-2/3, loop {LOOP_HZ:.0f} Hz.")
        # Gentle pre-ramp (0 -> START_W)
        if PRERAMP_S > 0.0 and START_W != 0.0:
            sweep(PRERAMP_S, 0.0, START_W)
        # Main sweep (START_W -> END_W)
        sweep(RAMP_TIME_S, START_W, END_W)
        print("Sweep complete.")
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        # Write CSV once (no I/O during loop)
        os.makedirs("data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = f"data/vel_torque_sweep_fastpdo_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "speed_cmd_rad_per_s", "speed_meas_rad_per_s", "torque_Nm"])
            for t_s, w_cmd, w_meas, tau in rows:
                w.writerow([f"{t_s:.3f}", f"{w_cmd:.6f}", f"{w_meas:.6f}", f"{tau:.6f}"])
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print(f"CSV saved: {path}")

if __name__ == "__main__":
    main()
