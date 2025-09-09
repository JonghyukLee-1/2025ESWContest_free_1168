#!/home/tunnel/jetson_project/yolov_env/bin/python

import rospy, time, sys, termios, tty, threading
from std_msgs.msg import Bool, String

from angle_calculate import (
    open_arduino, close_arduino, listen_from_arduino, get_angle
)

# ===== Constants / Pin Definitions =====
a1_port = "/dev/ttyACM0"    # Arm Arduino (servo)
a3_port = "/dev/ttyTHS1"    # Car Arduino (w,s,q, bNN)
baud    = 9600

# ===== Variables =====
A1 = None   # Servo handle
A3 = None   # Car handle
stop_event = False
manual_mode = False
thermal_triggered = False
mode_pub = None

# ===== Helpers (common) =====
def uart_send(ser, cmd: str):
    if ser is None:
        rospy.logwarn(f"[UART] handle=None, skip TX: {cmd}")
        return
    try:
        ser.write((cmd.strip() + "\n").encode("ascii"))
        rospy.loginfo(f"[UART] TX: {cmd}")
    except Exception as e:
        rospy.logwarn(f"[UART] send failed: {e}")

def wait_complete(ser, timeout=5.0, interval=0.1) -> bool:
    """Wait for 'COMPLETE' message from the serial port."""
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

# ===== Callbacks / Handlers =====
def thermal_callback(msg: Bool):
    """Emergency stop on thermal flag True + publish 'found' state."""
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
    """A1: for servo angle, A3: bNN command + wait for COMPLETE."""
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
    """Simple forward movement for a short duration."""
    if stop_event or manual_mode:
        return
    uart_send(A3, 'w')
    time.sleep(2.0)
    uart_send(A3, 'q')
    time.sleep(0.2)

def _get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def keyboard_listener():
    """m: toggle manual mode, space: stop(q), x: emergency stop, wasdq: movement"""
    global manual_mode, stop_event
    rospy.loginfo("[KEY] Listener started")
    while not rospy.is_shutdown():
        k = _get_key()
        if not k: continue
        if k == '\x03': break  # Ctrl-C
        key = k.lower()

        if key == 'm':
            manual_mode = not manual_mode
            rospy.loginfo(f"[KEY] manual_mode={manual_mode}")
            continue
        if key == ' ':
            uart_send(A3, 'q'); continue
        if key == 'x':
            stop_event = True
            uart_send(A3, 'q'); time.sleep(0.2); uart_send(A3, 'stop')
            rospy.logwarn("[KEY] Emergency stop"); continue

        if key in ('w','a','s','d','q'):
            if not manual_mode: manual_mode = True
            uart_send(A3, key)

def auto_mode_loop():
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

# ===== Main =====
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
