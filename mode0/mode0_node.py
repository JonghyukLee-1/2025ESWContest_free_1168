#!/home/tunnel/jetson_project/yolov_env/bin/python
# -*- coding: utf-8 -*-

import rospy, time, sys, termios, tty, threading
from std_msgs.msg import Bool, String

from angle_calculate import (
    open_arduino, close_arduino, listen_from_arduino, get_angle
)

# ===== UART configuration =====
a1_port = "/dev/ttyACM0"   # Arm Arduino (ultrasonic sensor + servo angle control)
a3_port = "/dev/ttyTHS1"   # Car Arduino (drive commands: w,s,q, bNN)
baud   = 9600

# ===== Global handles =====
A1 = None  # Arm Arduino handle
A3 = None  # Car Arduino handle
stop_event = False
manual_mode = False
thermal_triggered = False
mode_pub = None

# ===== UART helpers =====
def uart_send(ser, cmd: str):
    """Send a command string over UART (with newline)."""
    if ser is None:
        rospy.logwarn(f"[UART] handle=None, skip TX: {cmd}")
        return
    try:
        ser.write((cmd.strip() + "\n").encode("ascii"))
        rospy.loginfo(f"[UART] TX: {cmd}")
    except Exception as e:
        rospy.logwarn(f"[UART] send failed: {e}")

def wait_complete(ser, timeout=5.0, interval=0.1) -> bool:
    """
    Wait for a 'COMPLETE' response from Arduino.
    Returns True if received within timeout.
    """
    if ser is None: return False
    t0 = time.time()
    while time.time() - t0 < timeout and not stop_event and not rospy.is_shutdown():
        try:
            line = ser.readline().decode("ascii", "ignore").strip()
        except Exception:
            line = ""
        if line == "COMPLETE":
            return True
        time.sleep(interval)
    return False

# ===== Callbacks =====
def thermal_callback(msg: Bool):
    """
    Triggered when the thermal node detects a hot spot.
    Emergency stop + publish 'found'.
    """
    global thermal_triggered, stop_event, mode_pub
    if msg.data:
        thermal_triggered = True
        stop_event = True
        rospy.logwarn("[THERMAL] Emergency Stop")
        uart_send(A3, 'q'); time.sleep(0.2)
        uart_send(A3, 'stop')
        if mode_pub:
            mode_pub.publish("found")

def move_servo_auto():
    """
    Automatic servo control:
    - A1: read ultrasonic distance
    - Convert to angle
    - A3: send servo angle command (bNN) and wait for COMPLETE
    """
    if stop_event:
        return
    dist = listen_from_arduino(A1, max_wait=1.0)
    if dist is None:
        rospy.logwarn("[AUTO] dist=None"); time.sleep(0.02); return
    ang = get_angle(dist, pillar_height=63.0)
    rospy.loginfo(f"[AUTO] dist={dist:.2f} -> angle={ang}")
    uart_send(A3, f"b{ang}")
    ok = wait_complete(A3, timeout=5.0, interval=0.05)
    if not ok:
        rospy.logwarn("[AUTO] servo COMPLETE timeout or interrupted")

def move_car_forward():
    """
    Pre-steer once using a 3-sample median to suppress spikes,
    then do: 'w' for 2.0 s -> 'q' for 0.2 s.
    """
    if stop_event or manual_mode:
        return

    # Thresholds (tune as needed)
    NEAR_CM, FAR_CM, EMERGENCY_CM = 13.0, 16.0, 10.0
    STEER_PULSE_S = 1.0  # brief steer, then neutralize

    # --- burst: take 3 quick samples ---
    samples, emerg = [], 0
    for _ in range(3):
        d = listen_from_arduino(A1, max_wait=0.1)
        if d is not None:
            samples.append(d)
            if d <= EMERGENCY_CM: emerg += 1
        time.sleep(0.03)

    # Emergency: require 징횄2 confirmations
    if emerg >= 2:
        rospy.logwarn(f"[AUTO] EMERGENCY -> stop (samples={samples})")
        uart_send(A3, 'q'); time.sleep(0.1)
        uart_send(A3, 'stop')
        globals()['stop_event'] = True
        return

    # Pre-steer using median (if we have any sample)
    if samples:
        med = sorted(samples)[len(samples)//2]
        if med < NEAR_CM:
            rospy.loginfo(f"[AUTO] median={med:.2f} -> pre-steer RIGHT")
            uart_send(A3, 'd'); time.sleep(STEER_PULSE_S)
            uart_send(A3, 'q'); time.sleep(0.1)
        elif med > FAR_CM:
            rospy.loginfo(f"[AUTO] median={med:.2f} -> pre-steer LEFT")
            uart_send(A3, 'a'); time.sleep(STEER_PULSE_S)
            uart_send(A3, 'q'); time.sleep(0.1)
        else:
            rospy.loginfo(f"[AUTO] median={med:.2f} -> no pre-steer")
    else:
        rospy.logwarn("[AUTO] no distance sample -> no pre-steer")

    # Original forward window
    if not stop_event and not manual_mode and not thermal_triggered:
        uart_send(A3, 'w'); time.sleep(2.0)
        uart_send(A3, 'q'); time.sleep(0.2)

def _get_key():
    """Get one key press from stdin (non-blocking raw mode)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def keyboard_listener():
    """
    Keyboard teleop:
    - m: toggle manual_mode
    - space: stop (q)
    - x: emergency stop
    - wasdq: direct motor control
    """
    import sys, termios, tty, os
    global manual_mode, stop_event

    fd = sys.stdin.fileno()
    orig = termios.tcgetattr(fd)

    def restore():
        try: termios.tcsetattr(fd, termios.TCSADRAIN, orig)
        except: pass

    try:
        while not rospy.is_shutdown():
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            restore()

            if ch == '\x03': break
            key = ch.lower()

            if key == 'm':
                manual_mode = not manual_mode
                rospy.loginfo(f"[KEY] manual_mode={manual_mode}")
            elif key == ' ':
                uart_send(A3, 'q')
            elif key == 'x':
                stop_event = True
                uart_send(A3, 'q'); time.sleep(0.2); uart_send(A3, 'stop')
            elif key in ('w','a','s','d','q'):
                if not manual_mode: manual_mode = True
                uart_send(A3, key)
    finally:
        restore()
        os.system("stty sane")


def auto_mode_loop():
    """
    Automatic mode loop:
    - Move servo based on ultrasonic distance
    - Move car forward
    - Repeat until stop/manual/thermal event
    """
    rate = rospy.Rate(10)
    rospy.loginfo("[AUTO] Loop started")
    while not rospy.is_shutdown():
        if stop_event or manual_mode or thermal_triggered:
            rate.sleep(); continue
        move_servo_auto()
        if stop_event or manual_mode:
            rate.sleep(); continue
        move_car_forward()
        rate.sleep()

# ===== Main entry =====
def main():
    global A1, A3, mode_pub
    rospy.init_node('mode0_node', anonymous=True)

    # Open UART ports
    A1 = open_arduino(a1_port, baud)
    A3 = open_arduino(a3_port, baud)
    rospy.loginfo(f"[INIT] UART opened A1={a1_port}, A3={a3_port}")

    mode_pub = rospy.Publisher('/mode_result', String, queue_size=1)
    rospy.Subscriber('/thermal_flag_mode0', Bool, thermal_callback)
    threading.Thread(target=keyboard_listener, daemon=True).start()

    try:
        auto_mode_loop()
    finally:
        close_arduino(A1)
        close_arduino(A3)
        rospy.loginfo("[EXIT] mode0_node shutdown")

if __name__ == '__main__':
    main()
