 #!/home/tunnel/jetson_project/yolov_env/bin/python

import rospy, time, sys, termios, tty, threading

from std_msgs.msg import Bool, String
from smbus2 import SMBus, i2c_msg
from angle_calculate import listen_from_arduino, get_angle, close_arduino


i2c = None
I2C_ADDR = None
i2c_lock = threading.Lock()  
NBYTES_READ = 16              

mode_pub = None
stop_event = False
manual_mode = False
thermal_triggered = False

_MIN_TX_INTERVAL = 0.3
_last_tx = 0.0

def init_hardware():
    global i2c, I2C_ADDR
    bus_num = rospy.get_param('~i2c_bus', 1)
    I2C_ADDR = int(rospy.get_param('~i2c_addr', '0x06'), 16)
    try:
        rospy.loginfo(f"[INIT] Opening I2C bus {bus_num}, addr=0x{I2C_ADDR:02X}")
        i2c = SMBus(bus_num)
        time.sleep(0.05)
        rospy.loginfo("[INIT] I2C ready")

        step_ms = int(rospy.get_param('~servo_step_ms', 25))
        i2c_send(f"T{step_ms}")
    except Exception as e:
        rospy.logerr(f"[INIT] I2C open failed: {e}")
        i2c = None

def i2c_send(cmd: str):
    global _last_tx
    if i2c is None:
        rospy.logwarn(f"[I2C] unavailable, skip TX: {cmd}")
        return
    try:
        with i2c_lock:
            since = time.time() - _last_tx
            if since < _MIN_TX_INTERVAL:
                time.sleep(_MIN_TX_INTERVAL - since)

            data = str(cmd).encode('ascii')
            msg = i2c_msg.write(I2C_ADDR, data)
            i2c.i2c_rdwr(msg)
            _last_tx = time.time()
        rospy.loginfo(f"[I2C] TX: {cmd}")
    except Exception as e:
        rospy.logwarn(f"[I2C] write failed: {e}")

def i2c_read_status() -> str:
    if i2c is None:
        return ""
    try:
        with i2c_lock:
            msg = i2c_msg.read(I2C_ADDR, NBYTES_READ)
            i2c.i2c_rdwr(msg)
        raw = bytes(msg)
        s = raw.split(b'\x00', 1)[0].decode('ascii', 'ignore').strip()
        if s:
            rospy.logdebug(f"[I2C] RX: {s}")
        return s
    except Exception as e:
        rospy.logdebug(f"[I2C] read failed: {e}")
        return ""

def wait_complete(timeout=5.0, interval=0.1) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout and not stop_event:
        status = i2c_read_status()
        if status == "COMPLETE":
            return True
        time.sleep(interval)
    return False

def thermal_callback(msg: Bool):
    global thermal_triggered, stop_event, mode_pub
    if msg.data:
        thermal_triggered = True
        stop_event = True
        i2c_send('Q')
        time.sleep(0.2)
        i2c_send('STOP')
        rospy.logwarn("[THERMAL] Emergency Stop")
        close_arduino
        if mode_pub:
            mode_pub.publish("found")

def move_servo_auto():
    if stop_event:
        return
    dist = listen_from_arduino() #here, we get distance from ultrasound
    if dist is None:
        rospy.logwarn("[ULTRA] dist=None")
        time.sleep(0.02)
        return

    ang = get_angle(dist)  
    if ang is None:
        rospy.logwarn(f"[ANGLE] None (dist={dist})")
        time.sleep(0.02)
        return

    ang_i = int(round(ang))
    rospy.loginfo(f"[AUTO] dist={dist:.2f} -> angle={ang:.2f} -> TX b{ang_i}")
    i2c_send(f"b{ang_i}")

    ok = wait_complete(timeout=5.0, interval=0.05)
    if not ok:
        rospy.logwarn("[AUTO] servo COMPLETE timeout or interrupted")

def move_car_forward():
    if stop_event or manual_mode:
        return
    i2c_send('W')
    time.sleep(2.0)
    i2c_send('Q')
    time.sleep(0.2)

def auto_mode_loop():
    global manual_mode
    rate = rospy.Rate(10)
    rospy.loginfo("[AUTO] Loop started")
    while not rospy.is_shutdown():
        if stop_event or manual_mode or thermal_triggered:
            rate.sleep()
            continue
        move_servo_auto() # here, we move servo motor.
        if stop_event or manual_mode:
            rate.sleep()
            continue
        move_car_forward()
        rate.sleep()

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

'''
def keyboard_listener():
    global manual_mode
    rospy.loginfo("[KEY] Listener started")
    while not rospy.is_shutdown():
        key = get_key().upper()
        manual_mode = True
        if key in ['W','A','S','D','Q']:
            i2c_send(key)
        if key == '\x03':
            break
'''


def keyboard_listener():
    global manual_mode
    rospy.loginfo("[KEY] Listener started")
    while not rospy.is_shutdown():
        k = get_key()
        if not k:
            continue
        if k == '\x03':
            break
        key = k.upper()
        if key in ( 'W','A','S','D','Q'):
            if not manual_mode:
                manual_mode = True
            i2c_send(key)
                 
def main():
    global mode_pub # baisc setting : None
    rospy.init_node('mode0_node', anonymous=True)
    init_hardware()
    mode_pub = rospy.Publisher('/mode_result', String, queue_size=1)
    rospy.Subscriber('/thermal_flag_mode0', Bool, thermal_callback) # get if there is a center. if there is a center -> thermal_callback
    threading.Thread(target=keyboard_listener, daemon=True).start()
    try:
        auto_mode_loop()
    finally:
        try:
            if i2c is not None:
                i2c.close()
        except Exception:
            pass

if __name__=='__main__':
    main()
