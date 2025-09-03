#!/home/tunnel/jetson_project/yolov_env/bin/python

import rospy
import serial
import threading
import time
from std_msgs.msg import String, Float32, Int32

from arm_control import (go_mode, abnormal_mode, normal_mode, quit_mode, for_step_publish)
from angle_calculate import listen_from_arduino, close_arduino
from sound_data import sound_data

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

def param(name, default):
    return rospy.get_param("~" + name, default)

UART1_PORT = None
UART1_BAUD = 9600

I2C_BUS_NO = 1
I2C_ADDR2  = 0x18  # pillar/slide
I2C_ADDR3  = 0x06 # W/S drive (confirmed)

CENTER_DEADBAND = 3.0
CENTER_COOLDOWN = 0.15

manual_lock = threading.Lock()
manual_command = None

uart1_ser = None
i2c_bus   = None

last_center_tx = 0.0
aligned = False

def open_uart(port, baud, name):
    if not port:
        return None
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        rospy.loginfo(f"[{name}] open {port}@{baud}")
        return ser
    except Exception as e:
        rospy.logwarn(f"[{name}] open fail: {e}")
        return None

def i2c_send_text(addr, text):
    if i2c_bus is None:
        rospy.logwarn("[I2C] bus not available")
        return
    try:
        data = bytearray(text.encode('ascii'))[:31]
        payload = [c for c in data]
        i2c_bus.write_i2c_block_data(addr, 0x00, payload)
        rospy.loginfo(f"[I2C->0x{addr:02X}] {text}")
    except Exception as e:
        rospy.logwarn(f"[I2C->0x{addr:02X}] send fail: {e}")

'''
def center_callback(msg: Float32):
    global last_center_tx, aligned
    c = msg.data
    now = time.time()
    if abs(c) <= CENTER_DEADBAND:
        aligned = True
        return
    aligned = False
    if now - last_center_tx < CENTER_COOLDOWN:
        return
    cmd = "w" if c > 0 else "s"
    i2c_send_text(I2C_ADDR3, cmd)
    rospy.loginfo_throttle(1.0, f"[A3 I2C] {cmd} (center={c:.2f})")
    last_center_tx = now
''' # do we need this..?

def uart1_listener_thread():
    if uart1_ser is None:
        rospy.logwarn("[UART1] listener disabled (no port)")
        return
    while not rospy.is_shutdown():
        try:
            line = uart1_ser.readline().decode(errors='ignore').strip()
            if not line:
                time.sleep(0.05)
                continue
            u = line.upper()
            rospy.loginfo(f"[UART1 RX] {u}")
            if u == "STOP":
                rospy.loginfo("[ACTION] Slide movement complete")
            elif u == "HIT_READY":
                rospy.loginfo("[ACTION] Recording ready")
        except Exception as e:
            rospy.logwarn(f"[UART1 RX] error: {e}")
            time.sleep(0.1)

def x_control(topic='/have_to_move_x', i2c_addr=None, tx_cooldown=0.20, stop_hold=0.6, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR3
    prev_cmd = None
    last_tx  = 0.0
    stop_since = None
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, stop_since, done
        v = int(msg.data)
        cmd = 'w' if v > 0 else ('s' if v < 0 else 'q')
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'q':
            if stop_since is None:
                stop_since = now
            elif (now - stop_since) >= stop_hold:
                done = True
        else:
            stop_since = None
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done:
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'q')
    return True

def y_control(topic='/have_to_move_y', i2c_addr=None, tx_cooldown=0.20, stop_hold=0.6, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR2
    prev_cmd = None
    last_tx  = 0.0
    stop_since = None
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, stop_since, done
        v = int(msg.data)
        cmd = 'UP' if v > 0 else 'PILLAR_STOP'
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'PILLAR_STOP':
            if stop_since is None:
                stop_since = now
            elif (now - stop_since) >= stop_hold:
                done = True
        else:
            stop_since = None
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done:
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'PILLAR_STOP')
    return True

def run_mode1_sequence():
    ok_x = x_control()
    if not ok_x:
        rospy.logwarn("[MODE1] X-phase ended unexpectedly; continuing to Y-phase")
    ok_y = y_control()
    if not ok_y:
        rospy.logwarn("[MODE1] Y-phase ended unexpectedly")
    rospy.loginfo("[MODE1] sequence complete")
    return ok_x and ok_y

def main():
    global uart1_ser, i2c_bus
    rospy.init_node("mode1_node")

    global UART1_PORT, UART1_BAUD, I2C_BUS_NO, I2C_ADDR2, I2C_ADDR3
    UART1_PORT = param("uart1_port",  "/dev/ttyTHS1")
    UART1_BAUD = param("uart1_baud",  9600)
    I2C_BUS_NO = param("i2c_bus_no",  1)
    I2C_ADDR2  = param("arduino2_addr", 0x18)
    I2C_ADDR3  = param("arduino3_addr", 0x06)  # stays 0x04

    uart1_ser = open_uart(UART1_PORT, UART1_BAUD, "UART1")
    threading.Thread(target=uart1_listener_thread, daemon=True).start()


    if SMBus is not None:
        try:
            i2c_bus = SMBus(I2C_BUS_NO)
            rospy.loginfo(f"[I2C] bus {I2C_BUS_NO} ready, A2=0x{I2C_ADDR2:02X}, A3=0x{I2C_ADDR3:02X}")
        except Exception as e:
            rospy.logwarn(f"[I2C] open fail: {e}")
            i2c_bus = None
    else:
        rospy.logwarn("[I2C] smbus2 not installed; I2C control unavailable")

    ok = run_mode1_sequence()
    rospy.loginfo(f"mode 1 done, ok={ok}")

    go_mode(distance)

    sound_val = sound_data()  # 1: bad, 0: good, -1/None: invalid
    if sound_val == 1:
        sound_answer = "abnormal"
    elif sound_val == 0:
        sound_answer = "normal"
    else:
        sound_answer = None
        rospy.loginfo("no valid sound result")

    sound_pub = rospy.Publisher("/mode_result", String, queue_size=1)

    if sound_answer is not None:
        sound_pub.publish(String(data=sound_answer))
    else:
        rospy.logwarn("publish skipped (no valid result)")
    rospy.sleep(0.5)
    for_step_publish()


    rate = rospy.Rate(20)

    # this is for human control. now, we don't have a function to change manual_command, so this code never works.
    # when auto control doesn't work, we have to revive this code.
    
    try:
        '''
        while not rospy.is_shutdown():
            cmd = None
            with manual_lock:
                if manual_command:
                    cmd = manual_command
                    manual_command = None

            if cmd == "go":
                rospy.loginfo("[Manual] GO")
                go_mode(distance)
            elif cmd == "pillar_stop":
                i2c_send_text(I2C_ADDR2, "PILLAR_STOP")
            elif cmd == "quit":
                rospy.loginfo("[Manual] QUIT")
                quit_mode()
                break

            rate.sleep()
        '''
        pass
    finally:
        try: close_arduino()
        except: pass
        try:
            if uart1_ser and uart1_ser.is_open:
                uart1_ser.close()
        except: pass
        try:
            if i2c_bus: i2c_bus.close()
        except: pass
        rospy.loginfo("mode1_node exit")
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
