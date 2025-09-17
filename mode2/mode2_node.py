#!/home/tunnel/jetson_project/yolov_env/bin/python
# -*- coding: utf-8 -*-
"""
mode2_node.py

Purpose:
- Safe ABNORMAL retreat (ultrasonic-backed backoff)
- Pre-position to (r=0, theta=-90)
- Capture (r,theta) path via YOLO segmentation and follow it
- Return to (r=0, theta=-90)
- Publish "done" to /mode_result for mode_selector

Key changes in this version:
1) Segment-wise dynamic speeds from XY path spacing:
   - For each (r,theta) segment, compute an arc length s in XY (polar metric)
   - Given target XY speed v_xy_des, set segment time t = s / v
   - Command speeds:
        r_speed  = |짜횆r| / t      [cm/s]
        th_speed = |짜횆짜챔| / t      [deg/s]
   - Apply min/max caps separately to r and 짜챔 speeds

2) Hardware slowness calibration is applied only to SLEEP time:
   - r_time_scale and th_time_scale inflate the waiting time,
     so the firmware completes moves without blocking on unrealistic timing.
   - The commanded speeds still follow the path spacing policy.

3) Safer UART send (guard None), keyboard emergency-stop, and ROS params.
"""

import sys
import time
import math
import threading
import rospy
from std_msgs.msg import String

from camera_path import capture_path_rtheta
from arm_control import (
    open_serial, close_serial,
    abnormal_mode, quit_mode, uart_send, init_pub
)

# ==============================
# UART configuration
# ==============================
A1 = "/dev/ttyACM0"  # Arm Arduino (1: radial r, 2: theta)
A2 = "/dev/ttyACM1"  # Pillar Arduino (up/down/pillar_stop)
A3 = "/dev/ttyTHS1"  # Car   Arduino (drive: w/s/q, bxx)
UART_BAUD = 9600

# Serial handles (opened in main)
A1_SER = None
A2_SER = None
A3_SER = None

# ==============================
# Emergency handling
# ==============================
_emergency_evt = threading.Event()
ser_lock = threading.Lock()  # serialize UART writes

def emergency_stop(reason="keyboard"):
    """Hard stop for all controllable modules."""
    try:
        _emergency_evt.set()
        rospy.logwarn(f"[EMERGENCY] stop triggered ({reason})")
        try:
            if A1_SER:
                quit_mode(A1_SER)                # tell arm Arduino to quit current mode
            if A2_SER:
                uart_send(A2_SER, "pillar_stop") # stop pillar
            if A3_SER:
                uart_send(A3_SER, "q")           # car quit
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] UART notify failed: {e}")
    except Exception as e:
        rospy.logwarn(f"[EMERGENCY] unexpected error: {e}")

"""
def keyboard_listener_thread():
    #Type 'q' + Enter in this terminal to trigger emergency stop.
    rospy.loginfo("[KEY] Listener started (type 'q' + Enter to emergency-stop)")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05)
                continue
            if line.strip().lower() == "q":
                emergency_stop("keyboard:q")
        except Exception as e:
            rospy.logwarn(f"[KEY] error: {e}")
            time.sleep(0.1)
"""

# ==============================
# ROS param helper
# ==============================
def P(name, default):
    return rospy.get_param("~" + name, default)

# ==============================
# Runtime parameters (loaded in main)
# ==============================
# Target XY speed (used to split speeds between r and 짜챔 per segment)
v_xy_des_cm_s   = None  # [cm/s]

# Speed caps for r and 짜챔 (commanded to firmware)
r_speed_min     = None  # [cm/s]
r_speed_max     = None  # [cm/s]
th_speed_min    = None  # [deg/s]
th_speed_max    = None  # [deg/s]

# Hardware slowness calibration (applied to sleep time only)
r_time_scale    = None  # (>=1.0 징챈 slower hardware)
th_time_scale   = None  # (>=1.0 징챈 slower hardware)
wait_pillar = None      # (pillar up/down to capture the tile segment)

# Extra slack after each segment
safety_margin_s = None  # [s]

# Model & publishers
model_path      = None
pub_done        = None

# (Deprecated) legacy fixed speeds (kept for compatibility but unused in segment motion)
r_cmd_speed     = None
th_cmd_speed    = None

# ==============================
# UART helpers
# ==============================
def send_move(ser, motor_id, direction, speed, distance):
    """
    Send one move command to Arduino motor controller.

    Args:
        motor_id  : 1 for radial (r), 2 for theta
        direction : 'F' or 'B'
        speed     : speed value (cm/s for r, deg/s for 짜챔)
        distance  : magnitude (cm for r, deg for 짜챔)
    """
    if ser is None:
        rospy.logwarn("[move] serial handle is None; skip cmd")
        return
    cmd = f"{motor_id},{direction},{speed:.2f},{distance:.2f}"
    rospy.loginfo(f"[TX] {cmd}")
    with ser_lock:
        uart_send(ser, cmd)

def _segment_arc_len(r0, th0_deg, r1, th1_deg):
    """
    Compute "polar arc length" between (r0,짜챔0) and (r1,짜챔1) as an XY proxy.

    Using polar metric:
        s = sqrt( (짜횆r)^2 + (r_avg^2)(짜횆짜챔_rad^2) )

    Returns:
        s_cm    : arc length in [cm]
        dr     : r1 - r0 [cm]
        dth_deg: 짜챔1 - 짜챔0 [deg]
    """
    dr = float(r1) - float(r0)
    dth_deg = float(th1_deg) - float(th0_deg)
    dth_rad = math.radians(dth_deg)
    r_avg = 0.5 * (float(r0) + float(r1))
    s_cm = math.hypot(dr, r_avg * dth_rad)
    return s_cm, dr, dth_deg

# ==============================
# Motion core: segment-wise dynamic speed
# ==============================
def move(path, r_start, theta_start, visualize=False):
    """
    Follow a sequence of (r,짜챔) waypoints.

    Policy:
      - For each segment, compute polar arc length s.
      - With target XY speed v_xy_des_cm_s, set t_nom = s / v.
      - Command speeds:
            r_speed  = |짜횆r| / t_nom [cm/s]
            th_speed = |짜횆짜챔| / t_nom [deg/s]
      - Apply [min, max] caps separately for r and 짜챔.
      - Sleep time considers hardware slowness factors (r_time_scale, th_time_scale)
        plus a safety margin.
    """
    # Normalize to list
    if isinstance(path, tuple):
        path = [path]

    curr_r, curr_th = float(r_start), float(theta_start)

    for (r_next, th_next) in path:
        if _emergency_evt.is_set() or rospy.is_shutdown():
            break

        r_next  = float(r_next)
        th_next = float(th_next)

        # 1) Compute segment length and increments
        s_cm, dr, dth_deg = _segment_arc_len(curr_r, curr_th, r_next, th_next)
        if s_cm < 1e-6 and abs(dr) < 1e-6 and abs(dth_deg) < 1e-6:
            curr_r, curr_th = r_next, th_next
            continue

        # 2) Segment nominal duration (minimum clamp for numerics)
        v = max(float(v_xy_des_cm_s), 1e-6)
        t_nom = max(s_cm / v, 0.05)  # at least 50 ms for stability

        # 3) Command speeds from spacing
        r_speed_cmd  = abs(dr) / t_nom                      # [cm/s]
        th_speed_cmd = abs(dth_deg) / t_nom                 # [deg/s]

        # 4) Apply caps
        if r_speed_max > 0.0:
            r_speed_cmd = min(max(r_speed_cmd, r_speed_min), r_speed_max)
        if th_speed_max > 0.0:
            th_speed_cmd = min(max(th_speed_cmd, th_speed_min), th_speed_max)

        # 5) Determine directions
        dir_r  = 'F' if dr >= 0.0 else 'B'
        dir_th = 'F' if dth_deg >= 0.0 else 'B'

        # 6) Send motor commands (use absolute distance)
        if abs(dr) > 0.0:
            send_move(A1_SER, 1, dir_r,  r_speed_cmd,  abs(dr))
        if abs(dth_deg) > 0.0:
            send_move(A1_SER, 2, dir_th, th_speed_cmd, abs(dth_deg))

        # 7) Sleep long enough for both axes to realistically finish
        #    (only sleep time is scaled by hardware slowness)
        t_r  = (abs(dr)      / max(r_speed_cmd,  1e-6)) * float(r_time_scale) if abs(dr)      > 0.0 else 0.0
        t_th = (abs(dth_deg) / max(th_speed_cmd, 1e-6)) * float(th_time_scale) if abs(dth_deg) > 0.0 else 0.0
        time.sleep(max(t_r, t_th, 0.10) + float(safety_margin_s))

        # 8) Update pose
        curr_r, curr_th = r_next, th_next

    # NOTE: visualize option intentionally omitted (non-blocking autonomy)
    return curr_r, curr_th

# ==============================
# Cleanup and result publish
# ==============================
def publish_results_and_cleanup():
    """Gracefully stop modules and publish 'done'."""
    try:
        if A1_SER:
            quit_mode(A1_SER)
        if A2_SER:
            uart_send(A2_SER, "pillar_stop")
        if pub_done:
            pub_done.publish(String(data="done"))
    except Exception as e:
        rospy.logwarn(f"[mode2] cleanup error: {e}")

# ==============================
# Main behavior sequence
# ==============================
def run():
    try:
        # 1) ABNORMAL retreat (uses ultrasonic to safely back off)
        rospy.loginfo("[mode2] abnormal_mode()")
        abnormal_mode(A1_SER, should_stop=lambda: _emergency_evt.is_set())



        # 2) Small nudge to a capture-friendly pose (optional)
        uart_send(A2_SER, "down")
        move((8.0, 90.0), r_start=0.0, theta_start=90.0, visualize=False)
        rospy.sleep(wait_pillar)
        uart_send(A2_SER, "pillar_stop")

        # 3) Capture path via YOLO segmentation (non-blocking debug view)
        rospy.loginfo("[mode2] capturing path...")
        path = capture_path_rtheta(
            model_path=model_path,
            cam_index=0,
            conf_thr=0.7,
            step_px=8.0,
            cx=None, cy=None,
            px2cm_x=0.1,
            px2cm_y=0.1,
            device="cpu",
            timeout_open_s=10.0,
            max_attempts=10,
            timeout_detect_s=8.0,
            min_points=12,
            debug_view=True
        )

        # Return to center radius before starting (optional)
        uart_send(A2_SER, "up")
        move((0.0, 90.0), r_start=8.0, theta_start=90.0, visualize=False)
        rospy.sleep(wait_pillar)
        uart_send(A2_SER, "pillar_stop")

        if not path:
            rospy.logwarn("[mode2] empty path. cleanup.")
            return False

        # 4) Pre-position to (r=0, 짜챔=-90)
        move((0.0, 90.0), r_start=0.0, theta_start=-90.0, visualize=False)

        # 5) Follow captured path (assumed to start near 짜챔 = 90deg; adjust if needed)
        r_last, theta_last = move(path, r_start=0.0, theta_start=90.0, visualize=False)

        # 6) Return to (r=0, 짜챔=-90)
        move((0.0, -90.0), r_start=r_last, theta_start=theta_last, visualize=False)
        rospy.loginfo("[mode2] finished path following.")

        # Idle until shutdown or emergency
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and not _emergency_evt.is_set():
            r.sleep()

        return True

    finally:
        publish_results_and_cleanup()

# ==============================
# ROS node entry
# ==============================
def main():
    rospy.init_node("mode2_node")

    global v_xy_des_cm_s, r_speed_min, r_speed_max, th_speed_min, th_speed_max, wait_pillar
    global r_time_scale, th_time_scale, safety_margin_s, r_cmd_speed, th_cmd_speed
    global model_path, pub_done, A1_SER, A2_SER


    # ---- Load parameters (tunable at runtime) ----
    # Target XY speed for segment timing
    v_xy_des_cm_s = float(P("v_xy_des_cm_s", 2.5))  # [cm/s]

    # Speed caps
    r_speed_min  = float(P("r_speed_min", 0.5))     # [cm/s]
    r_speed_max  = float(P("r_speed_max", 1.4))     # [cm/s]
    th_speed_min = float(P("th_speed_min", 5.0))    # [deg/s]
    th_speed_max = float(P("th_speed_max", 10.0))   # [deg/s]

    # Hardware-time scaling (sleep only)
    r_time_scale  = float(P("r_time_scale", 1.3))
    th_time_scale = float(P("th_time_scale", 1.2))  # e.g., 180deg: ~21s vs ~6s 징챈 ~3.5x
    wait_pillar = float(P("wait_pillar", 10.0))

    # Extra slack per segment
    safety_margin_s = float(P("safety_margin_s", 0.3))

    # Model file path
    model_path = P("model_path", "/home/tunnel/Desktop/riot/best.pt")

    # (Deprecated) legacy fixed speeds (kept for compatibility but not used in segment motion)
    r_cmd_speed  = float(P("r_cmd_speed", 3.0))
    th_cmd_speed = float(P("th_cmd_speed", 30.0))

    # Publisher
    pub_done = rospy.Publisher("/mode_result", String, queue_size=1, latch=True)

    # Open UARTs
    A1_SER = open_serial(A1, UART_BAUD)
    A2_SER = open_serial(A2, UART_BAUD)
    # A3 (car) is optional in this mode; open if you need it:
    # A3_SER = open_serial(A3, UART_BAUD)

    # Optional: publish step counts from arm_control (already set there)
    init_pub()

    # Keyboard emergency listener
    #threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    rospy.on_shutdown(lambda: emergency_stop("shutdown"))

    try:
        if not _emergency_evt.is_set():
            ok = run()
            rospy.loginfo(f"mode 2 sequence done, ok={ok}")

        # Keep node alive for supervisor unless emergency/shutdown
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not _emergency_evt.is_set():
            r.sleep()

    finally:
        # Best-effort cleanup
        try:
            if A1_SER:
                close_serial(A1_SER)
            if A2_SER:
                close_serial(A2_SER)
            if A3_SER:
                close_serial(A3_SER)
        except Exception:
            pass
        rospy.loginfo("mode2_node exit")

if __name__ == "__main__":
    main()
    """
    except KeyboardInterrupt:
        emergency_stop("KeyboardInterrupt")
    """
