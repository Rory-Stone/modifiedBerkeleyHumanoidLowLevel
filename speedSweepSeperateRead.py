# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time, os, csv
from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# --------- user knobs ----------
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
# --------------------------------

def main():
    args = recoil.util.get_args()
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    dev = args.id

    # A slightly longer CAN RX timeout helps in VMs (if available)
    if hasattr(bus, "set_timeout_ms"):
        try:
            bus.set_timeout_ms(10)
        except Exception:
            pass

    # Velocity controller + safety
    if hasattr(bus, "write_velocity_kp"): bus.write_velocity_kp(dev, VEL_KP)
    if hasattr(bus, "write_velocity_ki"): bus.write_velocity_ki(dev, VEL_KI)
    if hasattr(bus, "write_torque_limit"): bus.write_torque_limit(dev, TORQUE_LIMIT_NM)

    bus.set_mode(dev, recoil.Mode.VELOCITY)
    bus.feed(dev)

    rate = RateLimiter(frequency=LOOP_HZ)

    pattern_len = CTRL_TICKS_PER_TORQUE + 1  # e.g., 3 control ticks, then 1 torque tick
    last_vel = None
    rows = []  # we buffer here, write once at the end
    t_start = time.time()

    def control_tick(w_cmd: float):
        """Send velocity command and read measured pos/vel via fast PDO."""
        nonlocal last_vel
        _pos, vel = bus.write_read_pdo_2(dev, 0.0, float(w_cmd))
        bus.feed(dev)
        if vel is not None:
            last_vel = float(vel)

    def torque_tick_and_buffer(w_cmd: float):
        """Separate tick: read torque only; buffer with last measured velocity."""
        # Keep heartbeat, add a small gap so this read doesn't collide with prior PDO
        bus.feed(dev)
        time.sleep(INTERFRAME_GAP_S)
        tau = None
        if hasattr(bus, "read_torque_measured"):
            try:
                tval = bus.read_torque_measured(dev)
                if tval is not None:
                    tau = float(tval)
            except Exception:
                return  # skip on timeout

        if (tau is not None) and (last_vel is not None):
            rows.append((
                time.time() - t_start,   # time_s
                float(w_cmd),            # speed_cmd_rad_per_s
                last_vel,                # speed_meas_rad_per_s
                tau                      # torque_Nm
            ))

    def sweep(duration_s: float, start_w: float, end_w: float):
        """Run a time-based sweep from start_w to end_w over duration_s."""
        t0 = time.time()
        tick = 0
        span = end_w - start_w
        while True:
            t = time.time() - t0
            frac = 1.0 if duration_s <= 0 else max(0.0, min(1.0, t / duration_s))
            w_cmd = start_w + span * frac

            slot = tick % pattern_len
            if slot < CTRL_TICKS_PER_TORQUE:
                control_tick(w_cmd)
            else:
                torque_tick_and_buffer(w_cmd)

            if frac >= 1.0:
                break
            tick += 1
            rate.sleep()

    try:
        print("Starting sweep …")
        # Gentle pre-ramp 0 -> START_W
        if PRERAMP_S > 0.0:
            sweep(PRERAMP_S, 0.0, START_W)
        # Main sweep START_W -> END_W
        sweep(RAMP_TIME_S, START_W, END_W)
        print("Sweep complete.")

    except KeyboardInterrupt:
        print("\n[Interrupted]")

    finally:
        # Write the CSV once (outside the timing-critical loop)
        os.makedirs("data", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = f"data/vel_torque_sweep_timesliced_{stamp}.csv"
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
