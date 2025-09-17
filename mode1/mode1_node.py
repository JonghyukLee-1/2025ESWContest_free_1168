#!/home/tunnel/jetson_project/yolov_env/bin/python
# pillar_modified_ver
import rospy
import threading
import time
import sys
import serial
from std_msgs import msg as std_msgs
from std_msgs.msg import String, Int32, Float32

# ===== UART Port Config =====
A1 = "/dev/ttyACM0"  # Arm Arduino(go/abnoraml/normal/quit/(1,F,30,5)/(2,F,30,90))
A2 = "/dev/ttyACM1"  # Pillar Arduino (up/down/pillar_stop)
A3 = "/dev/ttyTHS1"  # Car  Arduino (drive: w/s/q, bxx)
UART_BAUD = 9600

# Serial handles (open at startup)
A1_SER = None
A2_SER = None
A3_SER = None

# External helpers (already UART-based in your baseline)
from arm_control import (open_serial, close_serial, go_mode, quit_mode, uart_send, init_pub)
from sound_data import sound_data

# ===== Globals =====
sound_pub = None
pillar_time_pub = None
_emergency_evt = threading.Event()

def param(name, default):
    return rospy.get_param("~" + name, default)

# ---------- Emergency ----------
def emergency_stop(reason="keyboard"):
    """Hard stop everything we control (body + pillar), notify arm_control UART."""
    try:
        _emergency_evt.set()
        rospy.logwarn(f"[EMERGENCY] stop triggered ({reason})")
        try:
            if A1_SER: quit_mode(A1_SER)       
            if A2_SER: uart_send(A2_SER, "pillar_stop")  
            if A3_SER: uart_send(A3_SER, "q")            
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] UART notify failed: {e}")
    except Exception as e:
        rospy.logwarn(f"[EMERGENCY] unexpected error: {e}")

"""
def keyboard_listener_thread():
    #type 'q' + Enter to emergency-stop
    rospy.loginfo("[KEY] Listener started (type 'q' + Enter to emergency-stop)")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05); continue
            if line.strip().lower() == "q":
                emergency_stop("keyboard:q")
        except Exception as e:
            rospy.logwarn(f"[KEY] error: {e}")
            time.sleep(0.1)
"""

def x_control(topic='/have_to_move_x', tx_cooldown=0.20, rate_hz=20):
    """Drive body: ROS Int32 -> w/s/q over UART (BODY)."""
    prev_cmd, last_tx, done = None, 0.0, False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, done
        if _emergency_evt.is_set(): done = True; return
        v = int(msg.data)
        cmd = 'w' if v > 0 else ('s' if v < 0 else 'q')  # (Arduino basis)
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            uart_send(A3_SER, cmd)
            rospy.loginfo(f"[A3]-> {cmd}")
            prev_cmd, last_tx = cmd, now
        if cmd == 'q':
            done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        uart_send(A3_SER, 'q')
    return not _emergency_evt.is_set()

def y_control(topic='/have_to_move_y', ty_cooldown=0.20, rate_hz=20):
    """Move pillar: ROS Int32 -> up/pillar_stop over UART (PILLAR)."""
    prev_cmd, last_ty, done = None, 0.0, False
    up_start_time = None
    def cb(msg: Int32):
        nonlocal prev_cmd, last_ty, done, up_start_time
        if _emergency_evt.is_set(): done = True; return
        v = int(msg.data)
        # NOTE: Arduino basis (lowercase)
        if v > 0: cmd = 'up'
        # elif v < 0: cmd = 'down'  # enable when your flow needs it
        else: cmd = 'pillar_stop'
        now = time.time()
        if cmd == 'up' and up_start_time is None:
            up_start_time = now
        if cmd == 'pillar_stop' and up_start_time is not None:
            elapsed = now - up_start_time # here, up during time is recorded
            elapsed_2f = round(elapsed,2)
            rospy.loginfo(f"pillar up lasted {elapsed:.2f} sec")
            if pillar_time_pub is not None:
                pillar_time_pub.publish(Float32(data=elapsed_2f))
            up_start_time = None
            done = True
            
        if cmd != prev_cmd or (now - last_ty) >= ty_cooldown:
            uart_send(A2_SER, cmd)
            rospy.loginfo(f"[A2]-> {cmd}")
            prev_cmd, last_ty = cmd, now
        if cmd == 'pillar_stop':
            done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        uart_send(A2_SER, 'pillar_stop')
    return not _emergency_evt.is_set()

# ---------- Orchestration ----------
def run():
    # 1) X control
    ok_x = x_control()
    if not ok_x:
        rospy.logwarn("[MODE1] X-phase ended (possibly emergency); continuing")
    if _emergency_evt.is_set():
        return False

    uart_send(A3_SER, 'b00')
    time.sleep(2.0)

    try:
        _ = rospy.wait_for_message('/have_to_move_y', Int32, timeout=8.0)
    except rospy.ROSException:
        rospy.logerr("[MODE1] /have_to_move_y not received in 8s -> abort")
        emergency_stop("y_wait timeout")
        return False

    ok_y = y_control()
    if not ok_y:
        rospy.logwarn("[MODE1] Y-phase ended (possibly emergency)")
    rospy.loginfo("[MODE1] sequence complete")
    return (ok_x and ok_y and not _emergency_evt.is_set())

# ---------- Main ----------
def main():
    rospy.init_node("mode1_node")


    # Open UARTs
    global A1_SER, A2_SER, A3_SER, sound_pub, pillar_time_pub
    A1_SER = open_serial(A1, UART_BAUD)
    A2_SER = open_serial(A2, UART_BAUD)
    A3_SER = open_serial(A3, UART_BAUD)

    # Publisher for result
    if sound_pub is None:
        sound_pub = rospy.Publisher("/mode_result", String, queue_size=1)
        rospy.sleep(0.1)
    if pillar_time_pub is None:
        pillar_time_pub = rospy.Publisher("/pillar_up_time", Float32, queue_size=1)
        rospy.sleep(0.1)
    
    init_pub()

    # Start keyboard emergency listener
    #threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    rospy.on_shutdown(lambda: emergency_stop("shutdown"))


    try:
        should_stop = False

        if not _emergency_evt.is_set():
            ok = run()
            rospy.loginfo(f"mode 1 sequence done, ok={ok}")
            if rospy.is_shutdown() or _emergency_evt.is_set() or (not ok):
                should_stop = True

        if not _emergency_evt.is_set():
            try:
                # go_mode remains managed by arm_control (A1)
                go_mode(A1_SER, should_stop=lambda: _emergency_evt.is_set())
            except Exception as e:
                rospy.logwarn(f"go_mode failed: {e}")

        if not _emergency_evt.is_set():
            # Example HIT flow (A1 handled inside arm_control)
            uart_send(A1_SER, "HIT")
            # sound_val = sound_data()   # if you want the real classification
            # sound_answer = "abnormal" if sound_val == 1 else ("normal" if sound_val == 0 else None)
            sound_answer = "abnormal"
            if sound_answer is not None:
                sound_pub.publish(String(data=sound_answer))
            else:
                rospy.logwarn("publish skipped (no valid result)")
            rospy.sleep(0.5)

        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not _emergency_evt.is_set():
            r.sleep()
    finally:
        # Close UARTs
        try:
            if A1_SER: close_serial(A1_SER)
            if A2_SER: close_serial(A2_SER)
            if A3_SER: close_serial(A3_SER)
        except Exception:
            pass
        rospy.loginfo("mode1_node exit")

if __name__ == '__main__':
    main()
