import Jetson.GPIO as GPIO
import time
import rospy
import serial
import threading  # ADDED

from angle_calculate import listen_from_arduino
from std_msgs.msg import Int32

# === GPIO Setup ===
IN1, IN2, IN3, IN4 = 17, 18, 27, 22
# GPIO.setmode(GPIO.BCM)                               # CHANGED: remove eager init at import-time
# for pin in [IN1, IN2, IN3, IN4]:                     # CHANGED: remove eager init at import-time
#     GPIO.setup(pin, GPIO.OUT)
#     GPIO.output(pin, 0)

# ADDED: lazy init flags/lock
_GPIO_INITED = False
_GPIO_LOCK = threading.Lock()

def _ensure_gpio_ready():  # ADDED
    """Initialize GPIO once (lazy). Safe to call multiple times."""
    global _GPIO_INITED
    if _GPIO_INITED:
        return
    with _GPIO_LOCK:
        if _GPIO_INITED:
            return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in [IN1, IN2, IN3, IN4]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        _GPIO_INITED = True

def cleanup_gpio():  # ADDED
    """Release GPIO lines. Idempotent."""
    global _GPIO_INITED
    if not _GPIO_INITED:
        return
    with _GPIO_LOCK:
        try:
            for pin in [IN1, IN2, IN3, IN4]:
                try:
                    GPIO.output(pin, 0)
                except Exception:
                    pass
            GPIO.cleanup([IN1, IN2, IN3, IN4])
        except Exception:
            pass
        _GPIO_INITED = False

distance_cm = 999.0
moved_steps = 0

CM_PER_STEP = 1.0 / 450.0
DEFAULT_SPEED_CM_S = 2.0
DIST_THRESHOLD_GO = 5.0
DIST_THRESHOLD_ABNORMAL = 10.0

SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

def send_to_arduino(msg: str, port='/dev/ttyACM0'):
    baud = 9600
    try:
        with serial.Serial(port, baud, timeout=1) as ard:
            ard.write(f"{msg}\n".encode())
            rospy.loginfo(f"[TX->Arduino] {msg}")  # CHANGED: removed non-ASCII text
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")

'''
def distance_callback(msg):
    global distance_cm
    distance_cm = msg.data
'''

def move_motor(steps, delay=0.005, direction=1):
    _ensure_gpio_ready()  # ADDED
    seq = SEQ if direction == 1 else SEQ[::-1]
    for _ in range(steps):
        for pattern in seq:
            GPIO.output(IN1, pattern[0])
            GPIO.output(IN2, pattern[1])
            GPIO.output(IN3, pattern[2])
            GPIO.output(IN4, pattern[3])
            time.sleep(delay)

'''
def move_cm(cm, speed_cm_per_s=DEFAULT_SPEED_CM_S, direction=1):
    steps = int(abs(cm) / CM_PER_STEP)
    delay = max(0.002, 1.0 / (speed_cm_per_s * 450))
    move_motor(steps, delay=delay, direction=direction)
    return steps
'''

def cleanup():
    # CHANGED: ensure init before driving pins low (safe even if not inited)
    if not _GPIO_INITED:
        return
    for pin in [IN1, IN2, IN3, IN4]:
        try:
            GPIO.output(pin, 0)
        except Exception:
            pass

# === GO ===
def go_mode(dist_cm):
    global moved_steps
    rospy.loginfo("GO: Moving forward until distance < 5cm")
    send_to_arduino("GO")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    step = 1
    miss = 0
    while not rospy.is_shutdown():
        try:
            dist_cm = listen_from_arduino()
            if dist_cm is None:
                miss += 1
                if miss >= 100:
                    rospy.logwarn("no distance received")
                    break
                continue
            miss = 0
        except Exception as e:
            rospy.logwarn(f"listen_from_arduino() failed : {e}")
            break

        # NOTE: original code checks 'distance_cm' (global) here; kept as-is to minimize changes.
        if distance_cm < DIST_THRESHOLD_GO:
            rospy.loginfo("Distance < 5cm -> STOP")
            break
        move_motor(step, delay=delay, direction=1)
        moved_steps += step

    send_to_arduino("STOP")
    cleanup()

# === ABNORMAL ===
def abnormal_mode():
    global moved_steps
    rospy.loginfo("ABNORMAL: Moving backward until distance > 10cm")
    send_to_arduino("ABNORMAL")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    step = 1
    miss = 0
    while not rospy.is_shutdown():
        try:
            d = listen_from_arduino()
            if d is None:
                miss += 1
                if miss >= 100:
                    rospy.logwarn("no distance received")
                    break
                continue
            miss = 0
        except Exception as e:
            rospy.logwarn(f"listen_from_arduino() failed : {e}")
            break

        if d > DIST_THRESHOLD_ABNORMAL:
            rospy.loginfo("Distance > 10cm -> STOP")  # CHANGED: removed non-ASCII text
            break
        move_motor(step, delay=delay, direction=-1)
        moved_steps -= step

    send_to_arduino("STOP")
    cleanup()
    send_to_arduino("1,F,2.0,5")
    send_to_arduino("2,F,30.0,90")
    time.sleep(5)

# === NORMAL ===
def normal_mode():
    global moved_steps
    rospy.loginfo(f"NORMAL: Returning {moved_steps} steps forward")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    move_motor(moved_steps, delay=delay, direction=-1)
    moved_steps = 0
    rospy.loginfo("Return complete.")

# === QUIT ===
def quit_mode():
    rospy.loginfo("QUIT: Turning off all motors")
    send_to_arduino("QUIT")
    cleanup()
    cleanup_gpio()  # CHANGED: release GPIO lines instead of GPIO.cleanup() direct

def for_step_publish():
    step_pub = rospy.Publisher("/steps", Int32, queue_size=10)
    step_pub.publish(Int32(data=moved_steps))
