# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import math
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ------------ user knobs ------------
AMP_W = 10.0           # ramp between -AMP_W .. +AMP_W [rad/s]
PERIOD_S = 8.0         # seconds for a full up+down ramp
SAMPLE_HZ = 20.0       # loop rate (keep modest to avoid timeouts)
TAU_DECIMATE = 2       # read torque every N ticks (set to 1 for every tick)
VEL_KP = 0.2
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20
# ------------------------------------

def triangle_wave(t, period, amp):
    """Triangle from -amp → +amp → -amp over 'period' seconds."""
    phase = (t % period) / period  # 0..1
    if phase < 0.5:
        return -amp + 4.0 * amp * phase
    else:
        return +amp - 4.0 * amp * (phase - 0.5)

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # Set velocity loop + safety torque limit
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=SAMPLE_HZ)
    t0 = time.time()
    tick = 0
    last_tau = None

    print("Ramping speed between -{:.1f} and +{:.1f} rad/s; Ctrl+C to stop.".format(AMP_W, AMP_W))
    try:
        while True:
            t = time.time() - t0
            w_cmd = triangle_wave(t, PERIOD_S, AMP_W)

            # Send velocity command and get measured pos/vel (pos ignored here)
            _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
            bus.feed(dev)

            # Read torque occasionally to keep bus load low
            if tick % TAU_DECIMATE == 0 and hasattr(bus, "read_torque_measured"):
                try:
                    tau = bus.read_torque_measured(dev)
                    if tau is not None and math.isfinite(tau):
                        last_tau = float(tau)
                except Exception:
                    pass

            # Print like the MoveActuator demo
            if vel is not None and math.isfinite(vel):
                if last_tau is None or not math.isfinite(last_tau):
                    print(f"Measured vel: {vel:+.3f} rad/s\t torque: NA")
                else:
                    print(f"Measured vel: {vel:+.3f} rad/s\t torque: {last_tau:+.4f} N·m")

            tick += 1
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()

if __name__ == "__main__":
    main()
