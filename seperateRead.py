# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, math
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ---- user knobs ----
AMP_W = 10.0           # ramp between -AMP_W .. +AMP_W [rad/s]
PERIOD_S = 8.0         # seconds for a full up+down ramp
LOOP_HZ = 50.0         # main loop rate
CTRL_TICKS_PER_TORQUE = 3   # pattern: 3 control ticks, then 1 torque tick
VEL_KP = 0.2
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20
INTERFRAME_GAP_S = 0.003     # tiny gap before torque read
# --------------------

def triangle_wave(t, period, amp):
    """Triangle from -amp → +amp → -amp over 'period' seconds."""
    phase = (t % period) / period
    return (-amp + 4*amp*phase) if phase < 0.5 else ( +amp - 4*amp*(phase-0.5) )

def read_tau_separate_tick(bus, dev):
    """Read torque in a tick where we are NOT doing the pos/vel PDO."""
    # Prefer a fast PDO that returns (pos, torque) if available
    if hasattr(bus, "write_read_pdo_3"):
        try:
            # In VELOCITY mode, these write fields are ignored by the MCU,
            # and the reply is (position_measured, torque_measured).
            _pos, tau = bus.write_read_pdo_3(dev, 0.0, 0.0)
            if tau is not None:
                return float(tau)
        except Exception:
            pass
    # Fallback: parameter read
    if hasattr(bus, "read_torque_measured"):
        try:
            tau = bus.read_torque_measured(dev)
            if tau is not None:
                return float(tau)
        except Exception:
            pass
    return None

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # (If supported) slightly longer CAN RX timeout helps on VMs
    if hasattr(bus, "set_timeout_ms"):
        try: bus.set_timeout_ms(8)
        except Exception: pass

    # Velocity loop + safety
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    pattern_len = CTRL_TICKS_PER_TORQUE + 1  # e.g., 3 control ticks, then 1 torque tick
    tick = 0
    t0 = time.time()
    last_vel = None
    last_tau = None

    print(f"Time-sliced test: {CTRL_TICKS_PER_TORQUE} control ticks then 1 torque tick, loop {LOOP_HZ:.0f} Hz. Ctrl+C to stop.")
    try:
        while True:
            t = time.time() - t0
            w_cmd = triangle_wave(t, PERIOD_S, AMP_W)

            slot = tick % pattern_len

            if slot < CTRL_TICKS_PER_TORQUE:
                # ---- CONTROL TICK: do the fast pos/vel PDO ONLY ----
                _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
                bus.feed(dev)
                if vel is not None:
                    last_vel = float(vel)
            else:
                # ---- TORQUE TICK: do NOT call the pos/vel PDO this tick ----
                bus.feed(dev)  # keep heartbeat
                time.sleep(INTERFRAME_GAP_S)  # small gap so frames don't collide
                tau = read_tau_separate_tick(bus, dev)
                if tau is not None:
                    last_tau = tau

            # Print latest good values each loop (like MoveActuator style)
            if last_vel is not None:
                if last_tau is None:
                    print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: NA")
                else:
                    print(f"Measured vel: {last_vel:+.3f} rad/s\t torque: {last_tau:+.4f} N·m")

            tick += 1
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()

if __name__ == "__main__":
    main()
