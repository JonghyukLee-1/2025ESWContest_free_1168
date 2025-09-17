#!/home/tunnel/jetson_project/yolov_env/bin/python
# -*- coding: utf-8 -*-
"""
arm_control.py - Mode1 utilities (ARM/ultrasonic + stepper control)

Purpose:
- Provide unified helpers to open/close serial connections and send/receive UART messages
- Abstract motor/stepper controls (GO/ABNORMAL/QUIT modes) on top of serial and GPIO
- Serve as a reusable library for Mode1 (and possibly other modes) to avoid duplicated code

Dependencies:
- ROS (rospy, std_msgs/Int32)
- Jetson.GPIO
- pyserial

Main API:
# UART Helpers
- open_serial(port, baud=9600, timeout=0.2, wait_ready=True) -> serial.Serial
    Open and initialize a UART port, wait for Arduino reset/READY prompt
- close_serial(ser) -> None
- uart_send(ser, msg: str) -> bool
    Send one ASCII line with newline termination
- listen_from_arduino(ser, max_wait=1.0) -> Optional[float]
    Listen for a "DIST,xxx" message and parse distance in cm

# GPIO / Stepper Helpers
- move_motor(steps, delay=0.005, direction=1)
    Drive the stepper motor using 8-step half-stepping sequence
- cleanup_gpio()
    Reset GPIO pins and clean up
- publish_steps()
    Publish the cumulative step count to /steps (std_msgs/Int32)

# Mode Functions
- abnormal_mode(ser, should_stop=None, direction=1)
    Backward movement until ultrasonic distance exceeds threshold
    Supports soft pause and hard stop logic
- quit_mode(ser)
    Send QUIT command and clean up GPIO

# ROS Integration
- init_pub()
    Initialize publisher for /steps

Constants / Behavior:
- DIST_THRESHOLD_GO = 15.0 cm
- DIST_THRESHOLD_ABNORMAL = 20.0 cm
- Uses 8-step half-step sequence (e.g. 28BYJ-48 + ULN2003 driver)
- Reports moved steps via /steps topic
- Only handles Arduino1 (ultrasonic + arm stepper); Arduino2/3 are managed elsewhere
"""


import time, re, threading
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32

# ======================== UART Helpers ======================== #

_SERIAL_LOCK = threading.Lock()
_READY_PATTERNS = (b'READY', b'Ready', b'DIST,')
_NUM_RE = re.compile(r'^-?\d+(\.\d+)?$')

def _wait_ready_lines(ser, timeout=2.0):
    """Wait until Arduino sends a READY/DIST message (used after reset)."""
    t0, buf = time.time(), b""
    while time.time() - t0 < timeout:
        try:
            chunk = ser.read(64)
        except Exception:
            break
        if chunk:
            buf += chunk
            if any(p in buf for p in _READY_PATTERNS):
                return True
        time.sleep(0.02)
    return False

def open_serial(port: str, baud: int = 9600, timeout: float = 0.2, wait_ready: bool = True):
    """
    Open a UART serial port, auto-reset Arduino, flush buffers,
    and (optionally) wait for READY message.
    Returns: pyserial.Serial instance
    """
    import serial
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    time.sleep(2.0)  # allow Arduino auto-reset
    try:
        ser.reset_input_buffer(); ser.reset_output_buffer()
    except Exception:
        pass
    if wait_ready:
        _wait_ready_lines(ser, timeout=2.0)
    rospy.loginfo(f"[SERIAL] open {port}@{baud}")
    return ser

def close_serial(ser):
    """Close UART serial handle safely."""
    try:
        if ser and getattr(ser, "is_open", False):
            ser.close()
            rospy.loginfo("[SERIAL] closed")
    except Exception:
        pass

def uart_send(ser, msg: str) -> bool:
    """Send one line of ASCII text over UART (adds newline automatically)."""
    if ser is None or not getattr(ser, "is_open", True):
        rospy.logwarn("[SERIAL] handle unavailable; skip TX")
        return False
    data = (msg.strip() + "\n").encode("ascii", "ignore")
    with _SERIAL_LOCK:
        try:
            ser.write(data)
            ser.flush()
            port = getattr(ser, "port", "?")
            rospy.logdebug(f"[UART {port}] TX: {msg.strip()}")
            return True
        except Exception as e:
            rospy.logwarn(f"[{ser}] write fail: {e}")
            return False

def _drain(ser):
    """Drain any pending bytes in RX buffer before fresh read."""
    if ser is None: return
    try:
        n = getattr(ser, "in_waiting", 0)
        if n: ser.read(n)
    except Exception:
        pass

def _parse_distance(line: str):
    """
    Parse a distance line.
    Example: 'DIST,123.4' -> 123.4 (float, cm).
    Returns None if invalid.
    """
    if not line: return None
    s = line.strip()
    token = s.split(",", 1)[1].strip() if s[:5].upper() == "DIST," else s
    if not _NUM_RE.match(token): return None
    try:
        return float(token)
    except Exception:
        return None

def listen_from_arduino(ser, max_wait=1.0):
    """
    Listen once for distance reading from Arduino (cm).
    Returns float or None on timeout/invalid.
    """
    if ser is None:
        return None
    _drain(ser)
    t0 = time.time()
    while (time.time() - t0) < max_wait and not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            line = ""
        if not line:
            time.sleep(0.01); continue
        val = _parse_distance(line)
        if val is not None:
            rospy.loginfo(f"[DIST] {val:.2f} cm")
            return val
    return None

# =================== Jetson GPIO Stepper Helpers =================== #

IN1, IN2, IN3, IN4 = 17, 18, 27, 22
_GPIO_INITED = False
_GPIO_LOCK = threading.Lock()

def _ensure_gpio_ready():
    """Initialize GPIO pins for stepper (only once)."""
    global _GPIO_INITED
    if _GPIO_INITED: return
    with _GPIO_LOCK:
        if _GPIO_INITED: return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in (IN1, IN2, IN3, IN4):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        _GPIO_INITED = True

def cleanup_gpio():
    """Turn off stepper pins and cleanup."""
    global _GPIO_INITED
    if not _GPIO_INITED: return
    with _GPIO_LOCK:
        try:
            for pin in (IN1, IN2, IN3, IN4):
                try: GPIO.output(pin, GPIO.LOW)
                except Exception: pass
            GPIO.cleanup([IN1, IN2, IN3, IN4])
        except Exception:
            pass
        _GPIO_INITED = False

moved_steps = 0
DEFAULT_DELAY = 0.001
DIST_THRESHOLD_GO = 13.0        # stop threshold in GO mode (cm)
DIST_THRESHOLD_ABNORMAL = 14.0  # abnormal stop threshold (cm)

# 8-step half-stepping sequence
SEQ = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1],
]

step_pub = None

def move_motor(steps, delay=0.005, direction=1):
    """Rotate stepper by N steps (direction=+1 forward, -1 backward)."""
    _ensure_gpio_ready()
    seq = SEQ if direction == 1 else SEQ[::-1]
    for _ in range(steps):
        for a,b,c,d in seq:
            GPIO.output(IN1,a); GPIO.output(IN2,b)
            GPIO.output(IN3,c); GPIO.output(IN4,d)
            time.sleep(delay)

def publish_steps():
    """Publish moved_steps value over ROS /steps topic."""
    if step_pub is not None:
        step_pub.publish(Int32(data=moved_steps))

# ========================= Mode Implementations ========================= #

def abnormal_mode(ser, should_stop=None, direction=1):
    """
    ABNORMAL mode: move stepper backward until over threshold.
      - Uses ultrasonic distance from Arduino (listen_from_arduino)
    """
    global moved_steps

    rospy.loginfo("ABNORMAL: SoftPause + HardStop")
    uart_send(ser, "ABNORMAL")

    STEP_BLOCK  = 16
    START_DELAY = 0.0030
    MIN_DELAY   = 0.0025
    RAMP_FACTOR = 1.0
    POLL_EVERY  = 0.03
    MAX_MISS    = 100

    REQ_COUNT = 2
    below_cnt = 0
    paused = False

    step_delay = START_DELAY
    miss = 0
    last_poll = 0.0

    WARMUP_SEC = 0.7
    t0 = time.time()

    try:
        while not rospy.is_shutdown():
            if should_stop and should_stop():
                rospy.logwarn("[ABNORMAL] should_stop() -> STOP")
                break

            warmup = (time.time() - t0) < WARMUP_SEC

            if not paused:
                move_motor(STEP_BLOCK, delay=step_delay, direction=direction)
                moved_steps += STEP_BLOCK
                if step_delay > MIN_DELAY:
                    step_delay = max(MIN_DELAY, step_delay * RAMP_FACTOR)

            if warmup:
                continue

            if (time.time() - last_poll) >= POLL_EVERY:
                last_poll = time.time()
                try:
                    dist_cm = listen_from_arduino(ser, max_wait=1.0)
                except Exception as e:
                    rospy.logwarn(f"[ABNORMAL] listen_from_arduino(A1) failed: {e}")
                    paused = True
                    continue

                if dist_cm is None:
                    miss += 1
                    if miss >= MAX_MISS:
                        rospy.logwarn("[ABNORMAL] no distance; staying PAUSED")
                        paused = True
                        miss = 0
                    continue
                else:
                    miss = 0

                if dist_cm > DIST_THRESHOLD_ABNORMAL:
                    below_cnt += 1
                    if not paused:
                        rospy.loginfo(f"[ABNORMAL] {dist_cm:.2f}cm > thr -> Soft Pause")
                        paused = True
                    if below_cnt >= REQ_COUNT:
                        rospy.loginfo(f"[ABNORMAL] below thr {REQ_COUNT} times -> HARD STOP")
                        break
                else:
                    if paused:
                        rospy.loginfo(f"[ABNORMAL] recovered ({dist_cm:.2f}cm) -> Resume")
                    below_cnt = 0
                    paused = False

    finally:
        publish_steps()
        uart_send(ser, "STOP")
        cleanup_gpio()

def quit_mode(ser):
    """QUIT mode: send quit command and cleanup GPIO."""
    rospy.loginfo("QUIT: Turning off all motors")
    uart_send(ser, "QUIT")
    cleanup_gpio()

# =================== ROS Node init/shutdown =================== #

def init_pub():
    """Initialize ROS publisher for /steps."""
    global step_pub
    step_pub = rospy.Publisher("/steps", Int32, queue_size=10)
