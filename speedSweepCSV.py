# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, os, csv, math
import numpy as np
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ================== User knobs (safe defaults) ==================
START_W = -10.0           # start speed [rad/s]
END_W   = +10.0           # end speed   [rad/s]
PRERAMP_S = 3.0           # seconds to go 0 -> START_W (gentle spin-up)
RAMP_TIME_S = 120.0       # seconds to go START_W -> END_W (very slow)
SAMPLE_HZ = 20.0          # command & heartbeat rate (low to avoid VM/CAN timeouts)
TAU_DECIMATE = 2          # read torque every N ticks (~10 Hz if SAMPLE_HZ=20)
VELOCITY_KP = 0.2         # velocity loop Kp
VELOCITY_KI = 0.005       # velocity loop Ki
TORQUE_LIMIT_NM = 0.20    # joint torque cap (be conservative)
CURRENT_LIMIT_A = None    # e.g., 10.0 if your build exposes write_current_limit
K_T = None                # Nm/A; set to use tau = Kt * Iq (else use firmware torque)
# ================================================================

def sample_tau(bus, dev_id):
    """Prefer tau = Kt * Iq if K_T provided and Iq is available; else use firmware torque."""
    if K_T is not None:
        for name in ("read_iq", "read_motor_iq", "read_measured_iq"):
            if hasattr(bus, name):
                try:
                    iq = getattr(bus, name)(dev_id)
                    if iq is not None and math.isfinite(iq):
                        return float(iq) * float(K_T)
                except Exception:
                    pass
    if hasattr(bus, "read_torque_measured"):
        try:
            t = bus.read_torque_measured(dev_id)
            if t is not None and math.isfinite(t):
                return float(t)
        except Exception:
            pass
    return None

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # Controller setup
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VELOCITY_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VELOCITY_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)
    if CURRENT_LIMIT_A is not None and hasattr(bus, "write_current_limit"):
        bus.write_current_limit(dev, float(CURRENT_LIMIT_A))
    if hasattr(bus, "write_velocity_limit"):
        bus.write_velocity_limit(dev, max(abs(START_W), abs(END_W)) + 2.0)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=SAMPLE_HZ)

    # Prepare CSV output
    out_dir = "data"
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"vel_torque_ramp_{stamp}.csv")
    f = open(out_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["speed_rad_per_s", "torque_Nm"])

    print(f"Writing CSV to {out_path}")
    print(f"Pre-ramp {PRERAMP_S}s: 0 -> {START_W} rad/s, then ramp {RAMP_TIME_S}s: {START_W} -> {END_W} rad/s")

    # last torque value we successfully read (for continuous printing)
    last_tau_for_print = None

    try:
        # --------- gentle pre-ramp: 0 -> START_W ---------
        if PRERAMP_S > 0:
            t0 = time.time()
            tick = 0
            while True:
                t = time.time() - t0
                frac = min(max(t / PRERAMP_S, 0.0), 1.0)
                w_cmd = (1.0 - frac) * 0.0 + frac * START_W

                pos_meas, vel_meas = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
                bus.feed(dev)

                # read + log torque on decimated ticks
                if tick % TAU_DECIMATE == 0:
                    tau = sample_tau(bus, dev)
                    if tau is not None:
                        last_tau_for_print = tau
                    if (tau is not None) and (vel_meas is not None) and math.isfinite(vel_meas):
                        w.writerow([float(vel_meas), float(tau)])

                # ---- NEW: continuous console print like MoveActuator (vel + torque) ----
                if vel_meas is not None and math.isfinite(vel_meas):
                    if last_tau_for_print is None or not math.isfinite(last_tau_for_print):
                        print(f"Measured vel: {vel_meas:+.3f} rad/s\t torque: NA")
                    else:
                        print(f"Measured vel: {vel_meas:+.3f} rad/s\t torque: {last_tau_for_print:+.4f} N·m")

                if frac >= 1.0:
                    break
                tick += 1
                rate.sleep()

        # --------- main slow ramp: START_W -> END_W ---------
        t0 = time.time()
        tick = 0
        span = END_W - START_W
        while True:
            t = time.time() - t0
            frac = min(max(t / RAMP_TIME_S, 0.0), 1.0)
            w_cmd = START_W + span * frac

            pos_meas, vel_meas = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
            bus.feed(dev)

            # read + log torque on decimated ticks
            if tick % TAU_DECIMATE == 0:
                tau = sample_tau(bus, dev)
                if tau is not None:
                    last_tau_for_print = tau
                if (tau is not None) and (vel_meas is not None) and math.isfinite(vel_meas):
                    w.writerow([float(vel_meas), float(tau)])
                    # flush occasionally so you don't lose data if interrupted
                    if (tick // TAU_DECIMATE) % 50 == 0:
                        f.flush()

            # ---- NEW: continuous console print like MoveActuator (vel + torque) ----
            if vel_meas is not None and math.isfinite(vel_meas):
                if last_tau_for_print is None or not math.isfinite(last_tau_for_print):
                    print(f"Measured vel: {vel_meas:+.3f} rad/s\t torque: NA")
                else:
                    print(f"Measured vel: {vel_meas:+.3f} rad/s\t torque: {last_tau_for_print:+.4f} N·m")

            if frac >= 1.0:
                break
            tick += 1
            rate.sleep()

        print("Ramp complete.")

    except KeyboardInterrupt:
        print("\n[Interrupted] finishing up…")

    finally:
        f.flush(); f.close()
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print(f"CSV saved: {out_path}")

if __name__ == "__main__":
    main()
