#!/home/tunnel/jetson_project/yolov_env/bin/python
# -*- coding: utf-8 -*-
"""
arm_control.py ? Mode1 utilities: integrates with angle_calculate (get_angle function)

Main purpose:
- Provide unified helpers to open/close serial, send/receive UART messages
- Abstract motor/stepper controls (go/abnormal/quit modes) on top of serial and GPIO
- Act as a reusable library for Mode1 (and other modes) instead of duplicating code

Key functions:
  # UART Helpers
  open_serial(port, baud=9600, timeout=0.2, wait_ready=True) -> serial.Serial
  close_serial(ser) -> None
  uart_send(ser, msg: str) -> bool
  listen_from_arduino(ser, max_wait=1.0) -> Optional[float]   # returns distance in cm or None

  # Mode control / GPIO helpers
  init_node(node_name="motor_controller")
  shutdown_node()

  go_mode(ser, should_stop=None, direction=-1)
  abnormal_mode(ser)
  normal_mode()
  quit_mode(ser)

Arguments:
  - ser: Arduino1 handle (arm/ultrasonic; both distance measurement + stepper motor drive)
  - Arduino2/3 are managed elsewhere (Mode selector handles open/close). 
    Only A1-related helpers are provided here for Mode1 control.
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
DIST_THRESHOLD_GO = 15.0        # stop threshold in GO mode (cm)
DIST_THRESHOLD_ABNORMAL = 20.0  # abnormal stop threshold (cm)

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

def go_mode(ser, should_stop=None, direction=-1):
    """
    GO mode: move stepper forward until obstacle too close.
      - Uses ultrasonic distance from Arduino (listen_from_arduino)
      - Soft-pause if distance below threshold; hard-stop after N confirmations
    """
    global moved_steps

    rospy.loginfo("GO: SoftPause + HardStop")
    uart_send(ser, "GO")

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
                rospy.logwarn("[GO] should_stop() -> STOP")
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
                    rospy.logwarn(f"[GO] listen_from_arduino(A1) failed: {e}")
                    paused = True
                    continue

                if dist_cm is None:
                    miss += 1
                    if miss >= MAX_MISS:
                        rospy.logwarn("[GO] no distance; staying PAUSED")
                        paused = True
                        miss = 0
                    continue
                else:
                    miss = 0

                if dist_cm < DIST_THRESHOLD_GO:
                    below_cnt += 1
                    if not paused:
                        rospy.loginfo(f"[GO] {dist_cm:.2f}cm < thr -> Soft Pause")
                        paused = True
                    if below_cnt >= REQ_COUNT:
                        rospy.loginfo(f"[GO] below thr {REQ_COUNT} times -> HARD STOP")
                        break
                else:
                    if paused:
                        rospy.loginfo(f"[GO] recovered ({dist_cm:.2f}cm) -> Resume")
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
