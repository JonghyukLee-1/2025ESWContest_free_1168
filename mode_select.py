#!/home/tunnel/jetson_project/yolov_env/bin/python
import rospy, subprocess, time, signal, threading, os, atexit
from std_msgs.msg import String, Int32

thermal_proc = None
THERMAL_CMD = ["python3", "/home/tunnel/catkin_ws/src/mode0/src/thermal_code/thermal_module_ver.py"]

lock = threading.Lock()
current_result = None
active_mode = 0
p1 = p2 = None
m1_steps = None
m2_steps = None
pub_remain = None

_proc_registry = {}

def cb_result(msg: String):
    global current_result
    with lock:
        current_result = msg.data.strip()

def cb_steps(msg: Int32):
    global m1_steps, m2_steps, active_mode, p1, p2
    v = int(msg.data)
    if active_mode == 1 and p1 and p1.poll() is None:
        m1_steps = v
        rospy.loginfo(f"[MODE1] steps={v}")
    elif active_mode == 2 and p2 and p2.poll() is None:
        m2_steps = v
        rospy.loginfo(f"[MODE2] steps={v}")
    else:
        rospy.logwarn(f"[steps] late/unknown ignored (active={active_mode}, v={v})")

def start_node(cmd, name):
    rospy.loginfo(f"Starting: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    _proc_registry[proc] = name
    return proc

def stop_node(proc, name, t_int=3.0, t_term=2.0):
    if not proc or proc.poll() is not None:
        _proc_registry.pop(proc, None)
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        t0 = time.time()
        while proc.poll() is None and time.time()-t0 < t_int:
            time.sleep(0.1)
        if proc.poll() is None:
            os.killpg(pgid, signal.SIGTERM)
            t1 = time.time()
            while proc.poll() is None and time.time()-t1 < t_term:
                time.sleep(0.1)
        if proc.poll() is None:
            os.killpg(pgid, signal.SIGKILL)
    except Exception as e:
        rospy.logerr(f"stop {name} failed: {e}")
    finally:
        _proc_registry.pop(proc, None)

def stop_all_children():
    for proc, name in list(_proc_registry.items()):
        stop_node(proc, name)

def _cleanup(*_):
    stop_all_children()
    try:
        if rospy.core.is_initialized():
            rospy.signal_shutdown("mode_selector cleanup")
    except Exception:
        pass

signal.signal(signal.SIGINT, _cleanup)
signal.signal(signal.SIGTERM, _cleanup)
atexit.register(_cleanup)

def wait_result(timeout, allowed):
    global current_result
    with lock: current_result = None
    t0 = time.time()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        with lock:
            if current_result is not None and (current_result in allowed):
                v = current_result; current_result = None; return v
        if time.time() - t0 > timeout: return None
        r.sleep()

def publish_remain_and_reset():
    global m1_steps, m2_steps
    v1 = m1_steps if m1_steps is not None else 0
    v2 = m2_steps if m2_steps is not None else 0
    remain = v1 - v2
    pub_remain.publish(Int32(remain))
    rospy.loginfo(f"[MODE3 PUBLISH] /remain_steps={remain}")
    m1_steps = None; m2_steps = None

def main():
    global active_mode, p1, p2, pub_remain, m1_steps, m2_steps, thermal_proc
    rospy.init_node('mode_selector', anonymous=False)
    rospy.Subscriber('/mode_result', String, cb_result)
    rospy.Subscriber('/steps', Int32, cb_steps)
    pub_remain = rospy.Publisher('/remain_steps', Int32, queue_size=1, latch=True)

    thermal_proc = start_node(THERMAL_CMD, 'thermal_module')

    mode = 0 # basic setting
    while not rospy.is_shutdown():
        if mode == 0:
            p = start_node(['rosrun','mode0','mode0_node.py','__name:=mode0_node'], 'mode0')
            res = wait_result(180.0, {'found'}) # wait for 180s until it gets found.
            stop_node(p, 'mode0')
            mode = 1 if res == 'found' else 0

        elif mode == 1:
            with lock: active_mode = 1; m1_steps = None
            p1 = start_node(['rosrun','mode1','mode1_node_ver2.py','__name:=mode1_node'], 'mode1')
            res = wait_result(300.0, {'normal','abnormal'})
            t0 = time.time()
            while m1_steps is None and p1.poll() is None and time.time()-t0 < 1.0:
                time.sleep(0.05)
            if m1_steps is None: m1_steps = 0; rospy.logwarn("[MODE1] steps default=0")
            stop_node(p1, 'mode1'); p1 = None
            with lock: active_mode = 0
            if res == 'normal': mode = 3
            elif res == 'abnormal': mode = 2
            else: rospy.logwarn("[MODE1] timeout/unexpected ¡æ mode0"); mode = 0

        elif mode == 2:
            with lock: active_mode = 2; m2_steps = None
            p2 = start_node(['rosrun','mode2','mode2_node.py','__name:=mode2_node'], 'mode2')
            res = wait_result(180.0, {'done'})
            t0 = time.time()
            while m2_steps is None and p2.poll() is None and time.time()-t0 < 1.0:
                time.sleep(0.05)
            if m2_steps is None: m2_steps = 0; rospy.logwarn("[MODE2] steps default=0")
            stop_node(p2, 'mode2'); p2 = None
            with lock: active_mode = 0
            mode = 3

        elif mode == 3:
            p = start_node(['rosrun','mode3','mode3_node.py','__name:=mode3_node'], 'mode3')
            publish_remain_and_reset()
            time.sleep(0.2)
            res = wait_result(60.0, {'reset'})
            stop_node(p, 'mode3')
            if res != 'reset': rospy.logwarn("[MODE3] timeout/unexpected; continue")
            mode = 0

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
