# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ---- user knobs ----
SPEED_RAD_S = 8.0      # target constant speed [rad/s]
VEL_KP = 0.2           # velocity loop gains (adjust as needed)
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20 # safety cap
LOOP_HZ = 40.0         # loop rate (keep moderate to avoid CAN timeouts)
# --------------------

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # Configure controller
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    # Velocity mode
    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    print(f"Holding constant speed: {SPEED_RAD_S:+.3f} rad/s (Ctrl+C to stop)")
    try:
        while True:
            # In VELOCITY mode, position target is ignored; we pass 0.0
            pos_meas, vel_meas = bus.write_read_pdo_2(dev, 0.0, float(SPEED_RAD_S))
            bus.feed(dev)

            if (pos_meas is not None) and (vel_meas is not None):
                print(f"pos: {pos_meas:+.3f} rad\tvel: {vel_meas:+.3f} rad/s")

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()

if __name__ == "__main__":
    main()
