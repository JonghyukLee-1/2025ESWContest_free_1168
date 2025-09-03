#!/home/tunnel/jetson_project/yolov_env/bin/python
# arm_control.py — ArduinoLink 제거 버전 (전역 시리얼 1개 + 함수형)

import Jetson.GPIO as GPIO
import time
import rospy
import serial
import threading

from angle_calculate import listen_from_arduino
from std_msgs.msg import Int32

# =====================
# GPIO 설정
# =====================
IN1, IN2, IN3, IN4 = 17, 18, 27, 22

_GPIO_INITED = False
_GPIO_LOCK = threading.Lock()

def _ensure_gpio_ready():
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

def cleanup_gpio():
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

# =====================
# 전역 시리얼 (단일 핸들)
# =====================
_arduino = None
_serial_lock = threading.Lock()

def open_serial(port="/dev/ttyACM0", baud=9600, timeout=0.2):
    """시리얼을 한 번만 오픈해서 전역으로 보관."""
    global _arduino
    if _arduino is None or (not _arduino.is_open):
        _arduino = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(2.0)  # 보드 리셋 안정화
        try:
            _arduino.reset_input_buffer()
            _arduino.reset_output_buffer()
        except Exception:
            pass
        rospy.loginfo(f"[SERIAL] open {port}@{baud}")
    return _arduino

def current_serial():
    """현재 전역 시리얼 핸들 반환(없으면 None)."""
    return _arduino

def close_serial():
    """전역 시리얼 닫기(한 곳에서만 호출)."""
    global _arduino
    if _arduino and _arduino.is_open:
        try:
            _arduino.close()
            rospy.loginfo("[SERIAL] closed")
        except Exception:
            pass
    _arduino = None

def send_to_arduino(msg: str, ensure_open=True, port="/dev/ttyACM0", baud=9600, timeout=0.2):
    """쓰레드 세이프 TX. 필요 시 자동 오픈."""
    ser = current_serial()
    if ensure_open and (ser is None or not ser.is_open):
        ser = open_serial(port, baud, timeout)
    if ser is None or not ser.is_open:
        rospy.logwarn("[SERIAL] not open; skip send")
        return False
    data = (msg + "\n").encode()
    with _serial_lock:
        try:
            ser.write(data)
            ser.flush()
            rospy.loginfo(f"[TX->Arduino] {msg}")
            return True
        except Exception as e:
            rospy.logwarn(f"[SERIAL] write fail: {e}")
            return False

# =====================
# 모션/상수
# =====================
moved_steps = 0

CM_PER_STEP = 1.0 / 450.0
DEFAULT_DELAY = 0.001
DIST_THRESHOLD_GO = 15.0          # < 5cm 이면 정지 (전진)
DIST_THRESHOLD_ABNORMAL = 10.0   # > 20cm 이면 정지 (후진)

SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

# 런타임 전역
step_pub = None

def move_motor(steps, delay=0.005, direction=1):
    """direction=+1 전진, -1 후진"""
    _ensure_gpio_ready()
    seq = SEQ if direction == 1 else SEQ[::-1]
    for _ in range(steps):
        for pattern in seq:
            GPIO.output(IN1, pattern[0])
            GPIO.output(IN2, pattern[1])
            GPIO.output(IN3, pattern[2])
            GPIO.output(IN4, pattern[3])
            time.sleep(delay)

def publish_steps():
    if step_pub is not None:
        step_pub.publish(Int32(data=moved_steps))

# =====================
# GO: 전방 이동
# =====================
def go_mode(should_stop=None, serial_port="/dev/ttyACM0", baud=9600, timeout=0.2):
    """
    전방 이동: 거리 < 5cm면 정지
    should_stop: callable -> True면 즉시 정지(비상정지 연동)
    """
    global moved_steps, step_pub
    rospy.loginfo("GO: Moving forward until distance < 15cm")
    step_pub = rospy.Publisher("/steps", Int32, queue_size=10)
    send_to_arduino("GO", ensure_open=True, port=serial_port, baud=baud, timeout=timeout)

    STEP_BLOCK = 16
    START_DELAY = 0.0030
    MIN_DELAY = 0.0025
    RAMP_FACTOR = 1.0
    POLL_EVERY = 0.03
    MAX_MISS = 100

    REQ_COUNT = 5
    below_cnt = 0
    paused = False

    step_delay = START_DELAY
    miss = 0
    last_poll = 0.0

    WARMUP_SEC = 0.7
    t0 = time.time()

    ser = open_serial(serial_port, baud, timeout)

    try:
        while not rospy.is_shutdown():
            if should_stop and should_stop():
                rospy.logwarn("[GO] should_stop() -> STOP")
                break

            warmup = (time.time() - t0) < WARMUP_SEC

            if not paused:
                move_motor(STEP_BLOCK, delay=step_delay, direction=1)
                moved_steps += STEP_BLOCK
                if step_delay > MIN_DELAY:
                    step_delay = max(MIN_DELAY, step_delay * RAMP_FACTOR)

            now = time.time()
            if warmup:
                continue

            if (now - last_poll) >= POLL_EVERY:
                last_poll = now
                try:
                    dist_cm = listen_from_arduino(ser=ser)
                except Exception as e:
                    rospy.logwarn(f"[GO] listen_from_arduino() failed: {e}")
                    paused = True
                    continue

                if dist_cm is None:
                    miss += 1
                    if miss >= MAX_MISS:
                        rospy.logwarn("[GO] no distance received; staying PAUSED")
                        paused = True
                        miss = 0
                    continue
                else:
                    miss = 0

                if dist_cm < DIST_THRESHOLD_GO:
                    below_cnt += 1
                    if not paused:
                        rospy.loginfo(f"[GO] {dist_cm:.2f}cm < threshold -> Soft Pause")
                        paused = True
                    if below_cnt >= REQ_COUNT:
                        rospy.loginfo(f"[GO] below threshold {REQ_COUNT} times -> HARD STOP")
                        break
                else:
                    if paused:
                        rospy.loginfo(f"[GO] recovered ({dist_cm:.2f}cm) -> Resume")
                    below_cnt = 0
                    paused = False

    finally:
        publish_steps()
        send_to_arduino("STOP")
        cleanup_gpio()

# =====================
# QUIT: 종료 정리
# =====================
def quit_mode():
    """종료: 모든 모터 off + GPIO 해제"""
    rospy.loginfo("QUIT: Turning off all motors")
    send_to_arduino("QUIT", ensure_open=False)  # 이미 닫혀 있어도 무시됨
    cleanup_gpio()

def shutdown_node():
    try:
        cleanup_gpio()
    finally:
        close_serial()  # 전역 한 곳에서만 닫기
