# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, os, csv
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# --------- user knobs (safe defaults) ----------
START_W = -10.0          # start speed [rad/s]
END_W   = +10.0          # end speed   [rad/s]
PRERAMP_S   = 3.0        # seconds: 0 -> START_W (gentle)
RAMP_TIME_S = 120.0      # seconds: START_W -> END_W (slow sweep)

LOOP_HZ = 50.0           # main loop rate (control ticks)
CTRL_TICKS_PER_TORQUE = 3  # pattern: N control ticks, then 1 torque tick
INTERFRAME_GAP_S = 0.003   # small gap before torque read (~3 ms)

VEL_KP = 0.2
VEL_KI = 0.005
TORQUE_LIMIT_NM = 0.20
# -----------------------------------------------

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # (If supported) slightly longer RX timeout helps on VMs
    if hasattr(bus, "set_timeout_ms"):
        try: bus.set_timeout_ms(8)
        except Exception: pass

    # Velocity controller + safety
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    # CSV setup
    os.makedirs("data", exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = f"data/vel_torque_sweep_timesliced_{stamp}.csv"
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["time_s", "speed_cmd_rad_per_s", "speed_meas_rad_per_s", "torque_Nm"])

    pattern_len = CTRL_TICKS_PER_TORQUE + 1  # e.g., 3 control ticks, then 1 torque tick
    last_vel = None
    t_start = time.time()

    def control_tick(w_cmd):
        """Send velocity command and read measured pos/vel via fast PDO."""
        nonlocal last_vel
        _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
        bus.feed(dev)
        if vel is not None:
            last_vel = float(vel)

    def torque_tick_and_log(w_cmd):
        """On a separate tick: read torque only, then log using last_vel."""
        # Keep heartbeat, add a tiny gap so this read doesn't collide with PDO reply
        bus.feed(dev)
        time.sleep(INTERFRAME_GAP_S)
        tau = None
        if hasattr(bus, "read_torque_measured"):
            try:
                tval = bus.read_torque_measured(dev)
                if tval is not None:
                    tau = float(tval)
            except Exception:
                return  # skip if read failed

        # Log only when we have both a recent velocity and a torque sample
        if (tau is not None) and (last_vel is not None):
            t_now = time.time() - t_start
            w.writerow([f"{t_now:.3f}", f"{float(w_cmd):.6f}", f"{last_vel:.6f}", f"{tau:.6f}"])

    def sweep_duration(duration_s, start_w, end_w):
        """Run a time-based sweep from start_w to end_w over duration_s."""
        t0 = time.time()
        tick = 0
        span = end_w - start_w
        while True:
            t = time.time() - t0
            frac = max(0.0, min(1.0, t / max(duration_s, 1e-6)))
            w_cmd = start_w + span * frac

            slot = tick % pattern_len
            if slot < CTRL_TICKS_PER_TORQUE:
                control_tick(w_cmd)
            else:
                torque_tick_and_log(w_cmd)

            if frac >= 1.0:
                break
            tick += 1
            # Flush occasionally so data survives interruptions
            if tick % (pattern_len * 10) == 0:
                f.flush()
            rate.sleep()

    try:
        print(f"Saving CSV to {path}")
        # gentle pre-ramp 0 -> START_W
        if PRERAMP_S > 0.0:
            sweep_duration(PRERAMP_S, 0.0, START_W)
        # main sweep START_W -> END_W
        sweep_duration(RAMP_TIME_S, START_W, END_W)
        print("Sweep complete.")

    except KeyboardInterrupt:
        print("\n[Interrupted]")

    finally:
        f.flush(); f.close()
        bus.set_mode(dev, recoil.Mode.IDLE)
        bus.stop()
        print(f"CSV saved: {path}")

if __name__ == "__main__":
    main()
