# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import os, csv
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil

# ---- logging knobs ----
WRITE_IN_LOOP = False   # set True to stress-test CSV writes inside the loop
LOG_EVERY_N   = 1       # log every Nth tick (1 = log every tick)
OUT_DIR       = "data"
# -----------------------

args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)

device_id = args.id

kp = 0.2
kd = 0.005

frequency = 1.0  # motion frequency is 1 Hz
amplitude = 1.0  # motion amplitude is 1 rad

rate = RateLimiter(frequency=200.0)

bus.write_position_kp(device_id, kp)
bus.write_position_kd(device_id, kd)
bus.write_torque_limit(device_id, 0.2)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

# prep CSV (file created only when we stop unless WRITE_IN_LOOP=True)
os.makedirs(OUT_DIR, exist_ok=True)
stamp = time.strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(OUT_DIR, f"move_actuator_log_{stamp}.csv")

rows = []        # buffer for end-of-run write
f = None
writer = None

tick = 0
t0 = time.time()

try:
    if WRITE_IN_LOOP:
        f = open(out_path, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["time_s", "target_pos_rad", "measured_pos_rad", "measured_vel_rad_per_s"])

    while True:
        t_now = time.time()
        t_rel = t_now - t0

        target_angle = np.sin(2 * np.pi * frequency * t_now) * amplitude

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)

        if measured_position is not None and measured_velocity is not None:
            print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}")

            if tick % LOG_EVERY_N == 0:
                if WRITE_IN_LOOP:
                    writer.writerow([f"{t_rel:.6f}", f"{target_angle:.6f}",
                                     f"{measured_position:.6f}", f"{measured_velocity:.6f}"])
                    # optional: flush occasionally so you don't lose data on power loss
                    if (tick // LOG_EVERY_N) % 200 == 0:
                        f.flush()
                else:
                    rows.append((t_rel, target_angle, measured_position, measured_velocity))

        tick += 1
        rate.sleep()

except KeyboardInterrupt:
    pass
finally:
    # write CSV once at the end if we buffered
    if not WRITE_IN_LOOP:
        with open(out_path, "w", newline="") as f2:
            w = csv.writer(f2)
            w.writerow(["time_s", "target_pos_rad", "measured_pos_rad", "measured_vel_rad_per_s"])
            for t_rel, tgt, pos, vel in rows:
                w.writerow([f"{t_rel:.6f}", f"{tgt:.6f}", f"{pos:.6f}", f"{vel:.6f}"])
    else:
        if f is not None:
            f.flush()
            f.close()

    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
    print(f"CSV saved: {out_path}")
