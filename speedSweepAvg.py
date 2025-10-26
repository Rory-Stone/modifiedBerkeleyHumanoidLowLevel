# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import math
from collections import deque
import numpy as np

from loop_rate_limiters import RateLimiter
import berkeley_humanoid_lite_lowlevel.recoil as recoil


# -------------------- Setup --------------------
args = recoil.util.get_args()
bus = recoil.Bus(channel=args.channel, bitrate=1000000)
device_id = args.id

# Velocity-loop gains (position KP/KD are irrelevant in VELOCITY mode)
kp = 0.2
ki = 0.005
bus.write_velocity_kp(device_id, kp)
bus.write_velocity_ki(device_id, ki)

# Safety: keep torque within a conservative bound (adjust carefully)
bus.write_torque_limit(device_id, 0.2)

# Switch to velocity (speed) control mode
bus.set_mode(device_id, recoil.Mode.VELOCITY)
bus.feed(device_id)

# Command/heartbeat loop rate (steady-state test doesn't need to be fast)
SAMPLE_HZ = 50.0
rate = RateLimiter(frequency=SAMPLE_HZ)

# Sweep settings
v_min = -10.0
v_max =  10.0
v_step = 0.5
dwell_s = 5.0
speeds = np.arange(v_min, v_max + 1e-9, v_step)

# Averaging settings
FINAL_WINDOW_S = 1.5   # average the last ~1.5 s of steady samples
EPS_W_MIN = 0.2        # min velocity tolerance [rad/s] for "steady"
MIN_STEADY_S = 0.5     # require at least this many seconds of steady samples

# Optional: if you prefer τ = Kt * Iq, set your motor Kt (Nm/A) here; else leave None
K_T = None  # e.g., 0.0637

def robust_avg(values):
    """Median/MAD outlier rejection, then mean of inliers."""
    if not values:
        return float("nan")
    x = np.asarray(values, dtype=float)
    med = np.median(x)
    mad = np.median(np.abs(x - med)) * 1.4826  # ~robust sigma
    if mad == 0 or not np.isfinite(mad):
        return float(med)
    inliers = x[np.abs(x - med) <= 3.0 * mad]
    return float(np.mean(inliers)) if inliers.size else float(med)

def sample_tau(bus, dev_id):
    """Prefer Kt*Iq if K_T provided and Iq is available; else use torque_measured."""
    if K_T is not None:
        for name in ("read_iq", "read_motor_iq", "read_measured_iq"):
            if hasattr(bus, name):
                try:
                    iq = getattr(bus, name)(dev_id)
                    if iq is not None:
                        return float(iq) * float(K_T)
                except Exception:
                    pass
    # Fallback to the repo's measured torque parameter
    if hasattr(bus, "read_torque_measured"):
        try:
            val = bus.read_torque_measured(dev_id)
            if val is not None:
                return float(val)
        except Exception:
            return None
    return None

def read_torque_limit(bus, dev_id):
    try:
        if hasattr(bus, "read_torque_limit"):
            tl = bus.read_torque_limit(dev_id)
            return float(tl) if tl is not None else None
    except Exception:
        pass
    return None

# Results: (speed_rad_per_s, tau_ss_Nm, sat_flag)
samples = []

print(f"Starting velocity sweep from {v_min} to {v_max} rad/s in {v_step} rad/s steps; dwell {dwell_s:.1f}s per step.\n")

try:
    for v_cmd in speeds:
        # fresh limiter each step avoids "late" accumulation after a timeout
        rate = RateLimiter(frequency=SAMPLE_HZ)

        t0 = time.time()
        buf_tau = deque(maxlen=int(FINAL_WINDOW_S * SAMPLE_HZ))
        steady_time = 0.0

        while (time.time() - t0) < dwell_s:
            # In velocity mode, second arg is velocity target; first (position) is ignored.
            pos_meas, vel_meas = bus.write_read_pdo_2(device_id, 0.0, float(v_cmd))
            bus.feed(device_id)

            # Steady-state gate: require measured vel near command
            eps = max(EPS_W_MIN, 0.05 * abs(v_cmd))
            if (vel_meas is not None) and (abs(vel_meas - v_cmd) <= eps):
                tau_inst = sample_tau(bus, device_id)
                if (tau_inst is not None) and math.isfinite(tau_inst):
                    buf_tau.append(tau_inst)
                steady_time += 1.0 / SAMPLE_HZ
            else:
                # out of band -> reset timer so we only average truly steady periods
                steady_time = 0.0

            rate.sleep()

        tau_ss = float("nan") if steady_time < MIN_STEADY_S else robust_avg(list(buf_tau))

        # Optional: flag saturation if torque limit is available
        sat = False
        tl = read_torque_limit(bus, device_id)
        if (tl is not None) and math.isfinite(tau_ss) and abs(tau_ss) >= 0.9 * tl:
            sat = True

        print(f"v = {v_cmd:+5.1f} rad/s -> tau_ss = {tau_ss:+.4f} N·m" + ("  [SAT]" if sat else ""))
        samples.append((float(v_cmd), float(tau_ss) if math.isfinite(tau_ss) else float("nan"), sat))

except KeyboardInterrupt:
    print("\n[Interrupted] Printing collected data so far...\n")

finally:
    # Print final table (CSV-like)
    print("\n=== Velocity vs. Steady-State Torque (averaged) ===")
    print("speed_rad_per_s, torque_Nm, saturated")
    for v, tau, sat in samples:
        sat_str = "yes" if sat else "no"
        tau_str = f"{tau:.6f}" if math.isfinite(tau) else "NA"
        print(f"{v:.3f}, {tau_str}, {sat_str}")

    # Return the ESC to idle
    bus.set_mode(device_id, recoil.Mode.IDLE)
    bus.stop()
