# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import math
import struct
import os, csv
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# -------- user knobs --------
AMP_W = 10.0          # ramp between -AMP_W .. +AMP_W [rad/s]
PERIOD_S = 8.0        # seconds for a full up+down triangle wave
LOOP_HZ = 40.0        # main loop rate (alternate ticks: pos/vel, then torque)
VEL_KP = 0.2
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20
TIMEOUT_MS = 8        # CAN RX timeout if supported
PRINT_DECIMATE = 1    # print every N ticks (1 = every tick)
# ----------------------------

def triangle_wave(t, period, amp):
    """Triangle from -amp → +amp → -amp over 'period' seconds."""
    phase = (t % period) / period  # 0..1
    if phase < 0.5:
        return -amp + 4.0 * amp * phase
    else:
        return +amp - 4.0 * amp * (phase - 0.5)

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

def main():
    args = recoil.util.get_args()
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

    rate = RateLimiter(frequency=LOOP_HZ)

    t0 = time.time()
    tick = 0
    last_vel = None
    last_tau = None

    # Buffer rows; write once at the end (no disk I/O in-loop)
    rows = []  # (time_s, speed_meas_rad_per_s, torque_Nm)

    print(f"Ramping ±{AMP_W:.1f} rad/s (triangle); printing vel + torque via PDO fast path. Ctrl+C to stop.")
    try:
        while True:
            now = time.time()
            t = now - t0
            w_cmd = triangle_wave(t, PERIOD_S, AMP_W)

            if (tick % 2) == 0:
                # CONTROL TICK: one fast PDO for pos/vel
                _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
                bus.feed(dev)
                if vel is not None and math.isfinite(vel):
                    last_vel = float(vel)
            else:
                # TORQUE TICK: one fast PDO-3 for torque (no pos/vel request this tick)
                _p, tau = read_torque_pdo3(bus, dev, timeout_s=TIMEOUT_MS/1000.0)
                bus.feed(dev)
                if tau is not None and math.isfinite(tau):
                    last_tau = float(tau)
                    # Log when torque is fresh and we have a recent velocity
                    if last_vel is not None and math.isfinite(last_vel):
                        rows.append((now - t0, last_vel, last_tau))

            # Print like MoveActuator (continuous, using most recent valid samples)
            if (PRINT_DECIMATE == 1) or (tick % PRINT_DECIMATE == 0):
                if last_vel is not None and math.isfinite(last_vel):
                    if last_tau is None or not math.isfinite(last_tau):
                        print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: NA")
                    else:
                        print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: {last_tau:+.4f} N·m")

            tick += 1
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        # Write CSV once (no timing impact during run)
        os.makedirs("data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = f"data/vel_torque_triangle_fastpdo_{stamp}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "speed_meas_rad_per_s", "torque_Nm"])
            for t_s, v_meas, tau in rows:
                w.writerow([f"{t_s:.3f}", f"{v_meas:.6f}", f"{tau:.6f}"])

        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print(f"CSV saved: {path}")

if __name__ == "__main__":
    main()
