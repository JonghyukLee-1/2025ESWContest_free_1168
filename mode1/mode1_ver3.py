#!/home/tunnel/jetson_project/yolov_env/bin/python

import rospy
import threading
import time
import sys
from std_msgs.msg import String, Int32
from smbus2 import i2c_msg, SMBus

# arm_control에서만 시리얼을 열고/닫음
from arm_control import (init_node, go_mode, quit_mode, send_to_arduino, shutdown_node)
from sound_data import sound_data

def param(name, default):
    return rospy.get_param("~" + name, default)

I2C_BUS_NO = 0
I2C_ADDR2  = 0x18  # pillar/slide
I2C_ADDR3  = 0x08  # W/S drive (confirmed)

manual_lock = threading.Lock()
manual_command = None

# uart1_ser = None   # 제거
i2c_bus   = None

# === Emergency stop ===
_emergency_evt = threading.Event()

def emergency_stop(reason="keyboard"):
    """Hard stop everything we control."""
    try:
        _emergency_evt.set()
        rospy.logwarn(f"[EMERGENCY] stop triggered ({reason})")
        try: i2c_send_text(I2C_ADDR3, 'q')            # stop W/S drive
        except Exception as e: rospy.logwarn(f"[EMERGENCY] drive stop fail: {e}")
        try: i2c_send_text(I2C_ADDR2, 'PILLAR_STOP')  # stop pillar
        except Exception as e: rospy.logwarn(f"[EMERGENCY] pillar stop fail: {e}")
        try: quit_mode()
        except Exception as e: rospy.logwarn(f"[EMERGENCY] quit_mode failed: {e}")
        # UART 알림은 arm_control의 전역 핸들을 통해 전송 (여기서 새로 오픈하지 않음)
        try:
            send_to_arduino("STOP", ensure_open=False)  # 이미 열려있을 때만 전송
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] UART notify failed: {e}")
    except Exception as e:
        rospy.logwarn(f"[EMERGENCY] unexpected error: {e}")

def keyboard_listener_thread():
    """type 'st' + Enter to emergency-stop"""
    rospy.loginfo("[KEY] Listener started (type 'st' + Enter to emergency-stop)")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05); continue
            if line.strip().lower() == "st":
                emergency_stop("keyboard:st")
        except Exception as e:
            rospy.logwarn(f"[KEY] error: {e}")
            time.sleep(0.1)

_last_i2c_cmd = {}   # 주소별 마지막 전송 명령 저장

def i2c_send_text(addr, text):
    global _last_i2c_cmd
    if i2c_bus is None:
        rospy.logwarn("[I2C] bus not available"); return

    # 같은 명령 반복이면 무시
    if _last_i2c_cmd.get(addr) == text:
        return

    try:
        data = text.encode('ascii')[:31]  # Wire 32-byte limit
        msg = i2c_msg.write(addr, data)
        i2c_bus.i2c_rdwr(msg)
        rospy.loginfo(f"[I2C->0x{addr:02X}] {text}")
        _last_i2c_cmd[addr] = text
    except Exception as e:
        rospy.logwarn(f"[I2C->0x{addr:02X}] send fail: {e}")

# UART 리스너는 사용하지 않음(시리얼 읽기는 arm_control/angle_calculate 한 곳에서만)
# def uart1_listener_thread(): ...

def x_control(topic='/have_to_move_x', i2c_addr=None, tx_cooldown=0.20, rate_hz=20):
    if i2c_addr is None: i2c_addr = I2C_ADDR3
    prev_cmd, last_tx, done = None, 0.0, False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, done
        if _emergency_evt.is_set(): done = True; return
        v = int(msg.data)
        cmd = 'w' if v > 0 else ('s' if v < 0 else 'q')
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd, last_tx = cmd, now
        if cmd == 'q': done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'q')
    return not _emergency_evt.is_set()

def y_control(topic='/have_to_move_y', i2c_addr=None, tx_cooldown=0.20, rate_hz=20):
    if i2c_addr is None: i2c_addr = I2C_ADDR2
    prev_cmd, last_tx, done = None, 0.0, False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, done
        if _emergency_evt.is_set(): done = True; return
        v = int(msg.data)
        if v > 0: cmd = 'UP'
        # elif v < 0: cmd = 'DOWN'
        else: cmd = 'PILLAR_STOP'
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd, last_tx = cmd, now
        if cmd == 'PILLAR_STOP':
            done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'PILLAR_STOP')
    return not _emergency_evt.is_set()

def pre_y_seek(topic='/have_to_move_y', i2c_addr=None, tx_cooldown=0.20, rate_hz=20, timeout_sec=8.0):
    """
    Y 정렬 시작 전, 열화상 토픽이 뜰 때까지 주기적으로 'UP'을 전송.
    첫 메시지가 감지되면 'PILLAR_STOP'으로 멈추고 True 반환.
    timeout_sec <= 0 이면 타임아웃 없이 대기.
    """
    if i2c_addr is None: i2c_addr = I2C_ADDR2
    got_first = threading.Event()
    def probe_cb(_msg: Int32):
        try: got_first.set()
        except: pass
    sub = rospy.Subscriber(topic, Int32, probe_cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    start = time.time()
    last_tx = 0.0
    try:
        while (not rospy.is_shutdown()) and (not _emergency_evt.is_set()) and (not got_first.is_set()):
            now = time.time()
            if (now - last_tx) >= tx_cooldown:
                i2c_send_text(i2c_addr, 'UP')
                last_tx = now
            # 타임아웃 처리
            if timeout_sec > 0 and (now - start) >= timeout_sec:
                break
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        # 첫 메시지를 받았으면 정렬 인계 전 잠깐 정지
        if got_first.is_set():
            i2c_send_text(i2c_addr, 'PILLAR_STOP')
    return got_first.is_set()

def run_mode1_sequence():
    # 1) X 정렬
    ok_x = x_control()
    if not ok_x:
        rospy.logwarn("[MODE1] X-phase ended (possibly emergency); continuing")
    if _emergency_evt.is_set():
        return False

    # 2) 서보 0도 (b0) → 2초 대기
    i2c_send_text(I2C_ADDR3, 'b0')
    time.sleep(2.0)

    # 3) 열화상 토픽 대기하며 'UP' 유지
    timeout_sec = param("y_preseek_timeout", 8.0)  # 0 이면 무한 대기
    found = pre_y_seek(topic='/have_to_move_y', i2c_addr=I2C_ADDR2,
                       tx_cooldown=0.20, rate_hz=20, timeout_sec=timeout_sec)

    # === 타임아웃 처리: 프로그램 종료 ===
    if (timeout_sec > 0) and (not found):
        rospy.logerr(f"[MODE1] /have_to_move_y not received within {timeout_sec:.1f}s — aborting")
        # 안전 정지
        try: i2c_send_text(I2C_ADDR2, 'PILLAR_STOP')
        except: pass
        try: i2c_send_text(I2C_ADDR3, 'q')
        except: pass
        emergency_stop("y_preseek timeout")
        rospy.signal_shutdown("y_preseek timeout")  # ROS 노드 종료 신호
        return False

    # 4) Y 정렬
    if _emergency_evt.is_set():
        return False

    ok_y = y_control()
    if not ok_y:
        rospy.logwarn("[MODE1] Y-phase ended (possibly emergency)")
    rospy.loginfo("[MODE1] sequence complete")
    return (ok_x and ok_y and not _emergency_evt.is_set())

def main():
    global i2c_bus
    rospy.init_node("mode1_node")

    global I2C_BUS_NO, I2C_ADDR2, I2C_ADDR3
    I2C_BUS_NO = param("i2c_bus_no",  0)
    I2C_ADDR2  = param("arduino2_addr", 0x18)
    I2C_ADDR3  = param("arduino3_addr", 0x08)  # stays 0x08

    threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    # 시리얼 오픈은 arm_control 쪽에서만 수행 (여기서 절대 열지 않음)
    # uart1_ser = open_arduino(...)  # 삭제
    # threading.Thread(target=uart1_listener_thread, daemon=True).start()  # 사용 안 함
    try:
        if SMBus is not None:
            try:
                i2c_bus = SMBus(I2C_BUS_NO)
                rospy.loginfo(f"[I2C] bus {I2C_BUS_NO} ready, A2=0x{I2C_ADDR2:02X}, A3=0x{I2C_ADDR3:02X}")
            except Exception as e:
                rospy.logwarn(f"[I2C] open fail: {e}")
                i2c_bus = None
        else:
            rospy.logwarn("[I2C] smbus2 not installed; I2C control unavailable")

        if not _emergency_evt.is_set():
            ok = run_mode1_sequence()
            rospy.loginfo(f"mode 1 sequence done, ok={ok}")

        if not _emergency_evt.is_set():
            try:
                # go_mode 내부에서 거리 읽기는 arm_control 쪽 시리얼 핸들로 진행됨
                go_mode(lambda: _emergency_evt.is_set())
            except Exception as e:
                rospy.logwarn(f"go_mode failed: {e}")

        sound_pub = rospy.Publisher("/mode_result", String, queue_size=1, latch = True)
        if not _emergency_evt.is_set():
            sound_val = sound_data()  # 1: bad, 0: good, -1/None: invalid
            sound_answer = "abnormal" if sound_val == 1 else ("normal" if sound_val == 0 else None)
            
            if sound_answer is not None:
                sound_pub.publish(String(data=sound_answer))
            else:
                rospy.logwarn("publish skipped (no valid result)")
            rospy.sleep(0.5)

        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not _emergency_evt.is_set():
            r.sleep()
    finally:
        # close_arduino() 호출 금지: arm_control에서만 닫음
        if i2c_bus:
            try: i2c_bus.close()
            except: pass
        shutdown_node()
        rospy.loginfo("mode1_node exit")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        emergency_stop("KeyboardInterrupt")
