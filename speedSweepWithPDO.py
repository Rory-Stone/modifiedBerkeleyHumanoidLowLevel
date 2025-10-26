# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, os, csv, struct, math
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ------------------- user knobs -------------------
TAU_MIN = -0.20            # sweep start torque [N·m]
TAU_MAX = +0.20            # sweep end torque   [N·m]
PRERAMP_S = 2.0            # seconds: 0  -> TAU_MIN (gentle)
SWEEP_S   = 60.0           # seconds: TAU_MIN -> TAU_MAX (slow)
LOOP_HZ   = 40.0           # one PDO-3 round-trip per tick
VEL_LP_TC = 0.10           # velocity low-pass time constant [s]
TORQUE_LIMIT_NM = 0.22     # safety limit (>= |TAU_MAX|, but still conservative)
# --------------------------------------------------

def send_pdo3_and_get(bus, dev, pos_target: float, tau_target: float, timeout_s=0.008):
    """
    Fast frame:
      TX: RECEIVE_PDO_3  [position_target, torque_target]
      RX: TRANSMIT_PDO_3 [position_measured, torque_measured]
    """
    bus.transmit(recoil.CANFrame(
        dev,
        recoil.Function.RECEIVE_PDO_3,
        size=8,
        data=struct.pack("<ff", float(pos_target), float(tau_target))
    ))
    rx = bus.receive(
        filter_device_id=dev,
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

    # Slightly longer RX timeout helps on VMs
    if hasattr(bus, "set_timeout_ms"):
        try: bus.set_timeout_ms(10)
        except Exception: pass

    # Choose a torque/current mode if available; fall back gracefully
    target_mode = None
    for name in ("TORQUE", "CURRENT", "IQ"):  # try common names
        if hasattr(recoil.Mode, name):
            target_mode = getattr(recoil.Mode, name)
            break
    if target_mode is None:
        target_mode = recoil.Mode.VELOCITY  # fallback; MCU may ignore tau_target in this mode

    # Safety cap: make sure we can't exceed the requested sweep magnitude
    if hasattr(bus, "write_torque_limit"):
        bus.write_torque_limit(dev, float(TORQUE_LIMIT_NM))

    bus.set_mode(dev, target_mode)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    # Buffer samples in memory; write CSV once at the end
    rows = []  # (time_s, tau_cmd_Nm, tau_meas_Nm, vel_meas_rad_s, pos_meas_rad)

    # State for velocity from position (finite difference + low-pass)
    prev_t = None
    prev_pos = None
    vel_lp = 0.0

    t_start = time.time()

    def run_segment(duration_s: float, start_tau: float, end_tau: float):
        nonlocal prev_t, prev_pos, vel_lp
        t0 = time.time()
        while True:
            now = time.time()
            t = now - t0
            if duration_s <= 0.0:
                frac = 1.0
            else:
                frac = max(0.0, min(1.0, t / duration_s))

            tau_cmd = start_tau + (end_tau - start_tau) * frac

            # Single fast round-trip this tick
            pos_meas, tau_meas = send_pdo3_and_get(bus, dev, 0.0, tau_cmd)
            bus.feed(dev)

            # Estimate velocity from position (no extra PDO needed)
            vel_est = None
            if (pos_meas is not None):
                if prev_t is not None and prev_pos is not None:
                    dt = now - prev_t
                    if dt > 0:
                        vel_inst = (pos_meas - prev_pos) / dt
                        # 1st-order low-pass: alpha = dt/(TC+dt)
                        alpha = dt / (VEL_LP_TC + dt)
                        vel_lp = (1 - alpha) * vel_lp + alpha * vel_inst
                        vel_est = vel_lp
                prev_t = now
                prev_pos = pos_meas

            # Buffer only valid samples
            if (tau_meas is not None) and (vel_est is not None):
                rows.append((
                    now - t_start,
                    float(tau_cmd),
                    float(tau_meas),
                    float(vel_est),
                    float(pos_meas)
                ))

            if frac >= 1.0:
                break

            rate.sleep()

    try:
        print(f"Torque sweep: {TAU_MIN:+.3f} -> {TAU_MAX:+.3f} N·m at {LOOP_HZ:.0f} Hz (fast PDO-3). Ctrl+C to stop.")
        # Gentle pre-ramp 0 -> TAU_MIN
        if PRERAMP_S > 0.0 and TAU_MIN != 0.0:
            run_segment(PRERAMP_S, 0.0, TAU_MIN)
        # Main sweep TAU_MIN -> TAU_MAX
        run_segment(SWEEP_S, TAU_MIN, TAU_MAX)
        print("Sweep complete.")

    except KeyboardInterrupt:
        print("\n[Interrupted]")

    finally:
        # Write CSV once (no I/O inside loop)
        os.makedirs("data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = f"data/torque_sweep_fastpdo_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s","torque_cmd_Nm","torque_meas_Nm","speed_meas_rad_per_s","position_rad"])
            for t_s, tau_cmd, tau_meas, w_meas, pos in rows:
                w.writerow([f"{t_s:.3f}", f"{tau_cmd:.6f}", f"{tau_meas:.6f}", f"{w_meas:.6f}", f"{pos:.6f}"])

        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print(f"CSV saved: {path}")

if __name__ == "__main__":
    main()
