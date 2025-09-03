#!/home/tunnel/jetson_project/yolov_env/bin/python
# mode3_reset_node.py ? normal -> normal_mode(steps) -> down -> pillar_stop -> publish('reset')
import time
import threading
import serial
from smbus2 import SMBus

import rospy
from std_msgs.msg import String, Int32, Bool

def P(name, default):
    return rospy.get_param("~" + name, default)

class Arduino1Serial:
    def __init__(self, port, baud, timeout=1.0):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2.0)
        rospy.loginfo(f"[mode3] Serial opened: {self.port} @ {self.baud}")

    def send_line_retry(self, line: str, retries=3, backoff=0.2):
        msg = (line.strip() + "\n").encode("utf-8")
        for i in range(retries):
            try:
                self.ser.write(msg)
                self.ser.flush()
                rospy.loginfo(f"[mode3] -> Arduino1(serial): {line.strip()} (try {i+1})")
                return True
            except Exception as e:
                rospy.logwarn(f"[mode3] serial send failed({i+1}/{retries}): {e}")
                time.sleep(backoff * (i+1))
        return False

    def close(self):
        try:
            if self.ser: self.ser.close()
        except: pass

class Arduino2I2C:
    def __init__(self, bus_id: int, addr: int):
        self.bus_id, self.addr = bus_id, addr
        self.bus = None

    def open(self):
        self.bus = SMBus(self.bus_id)
        rospy.loginfo(f"[mode3] I2C opened: bus={self.bus_id}, addr=0x{self.addr:02X}")

    def send_text_retry(self, text: str, retries=3, backoff=0.2):
        data = list(text.encode("utf-8"))
        if len(data) > 32:
            rospy.logwarn("[mode3] I2C payload >32 bytes; truncating")
            data = data[:32]
        for i in range(retries):
            try:
                self.bus.write_i2c_block_data(self.addr, 0x00, data)
                rospy.loginfo(f"[mode3] -> Arduino2(I2C): {text} (try {i+1})")
                return True
            except Exception as e:
                rospy.logwarn(f"[mode3] i2c send failed({i+1}/{retries}): {e}")
                time.sleep(backoff * (i+1))
        return False

    def close(self):
        try:
            if self.bus: self.bus.close()
        except: pass

class Mode3ResetNode:
    def __init__(self):
        # ==== Params ====
        self.serial_port = P("serial_port", "/dev/ttyACM0")  # Arduino1
        self.serial_baud = P("serial_baud", 9600)
        self.i2c_bus_id  = P("i2c_bus_id", 1)               # Jetson/RPi usually 1
        self.i2c_addr    = P("i2c_addr", 0x18)              # Arduino2
        self.i2c_retries = int(P("i2c_retries", 3))
        self.ser_retries = int(P("serial_retries", 3))

        self.move_done_timeout  = float(P("move_done_timeout", 20.0))
        self.after_down_sleep_s = float(P("after_down_sleep_s", 3.0))

        self.remain_steps_topic = P("remain_steps_topic", "/remain_steps")
        self.move_cmd_topic     = P("move_cmd_topic", "/mode3/move_steps")
        self.move_done_topic    = P("move_done_topic", "/mode3/move_done")
        self.mode_result_topic  = P("mode_result_topic", "/mode_result")

        # ==== State ====
        self._lock = threading.Lock()
        self._remain_steps = None
        self._consumed_steps = False
        self._move_done = False
        self._ran = False  # idempotent guard

        # ==== ROS IO ====
        self.pub_move_cmd = rospy.Publisher(self.move_cmd_topic, Int32, queue_size=1, latch=True)
        self.pub_mode_result = rospy.Publisher(self.mode_result_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.remain_steps_topic, Int32, self._cb_remain_steps)
        rospy.Subscriber(self.move_done_topic, Bool, self._cb_move_done)

        # ==== HW ====
        self.ard1 = Arduino1Serial(self.serial_port, self.serial_baud)
        self.ard2 = Arduino2I2C(self.i2c_bus_id, self.i2c_addr)

    # ---------- Callbacks ----------
    def _cb_remain_steps(self, msg: Int32):
        with self._lock:
            if self._consumed_steps:
                # ÀÌ¹Ì ¼ÒºñÇßÀ¸¸é ¹«½Ã (Áßº¹ ½ÇÇà ¹æÁö)
                return
            self._remain_steps = int(msg.data)
            rospy.loginfo(f"[mode3] received /remain_steps = {self._remain_steps} (latched)")

    def _cb_move_done(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._move_done = True
            rospy.loginfo("[mode3] move_done TRUE")

    # ---------- Helpers ----------
    def _wait_first_steps(self, timeout=15.0):
        start = time.time()
        while not rospy.is_shutdown():
            with self._lock:
                if self._remain_steps is not None:
                    self._consumed_steps = True
                    val = self._remain_steps
                    rospy.loginfo(f"[mode3] steps locked = {val}")
                    return val
            if time.time() - start > timeout:
                rospy.logwarn("[mode3] remain_steps not received; default 0")
                return 0
            rospy.sleep(0.05)

    def _normal_mode_move(self, steps: int):
        rospy.loginfo(f"[mode3] normal_mode: publish move_steps={steps}")
        self.pub_move_cmd.publish(Int32(data=steps))
        if not self.wait_move_done:
            return True
        # wait with timeout
        with self._lock:
            self._move_done = False
        start = time.time()
        while not rospy.is_shutdown() and (time.time() - start) < self.move_done_timeout:
            with self._lock:
                if self._move_done:
                    return True
            rospy.sleep(0.05)
        rospy.logwarn("[mode3] move_done timeout; continue anyway")
        return False

    # ---------- Flow ----------
    def run_once(self):
        with self._lock:
            if self._ran:
                rospy.logwarn("[mode3] already executed; ignoring re-entry")
                return
            self._ran = True

        self.ard1.open()
        self.ard2.open()

        # 1) steps È®º¸(1È¸ ¼Ò¸ð)
        steps = self._wait_first_steps(timeout=15.0)

        # 2) Arduino1¿¡ normal
        ok_serial = self.ard1.send_line_retry("normal", retries=self.ser_retries)
        if not ok_serial:
            rospy.logwarn("[mode3] failed to send 'normal' over serial; continuing")

        # 3) normal_mode ÀÌµ¿
        self._normal_mode_move(steps)

        # 4) Arduino2¿¡ down -> ´ë±â -> pillar_stop
        ok_i2c1 = self.ard2.send_text_retry("down", retries=self.i2c_retries)
        if not ok_i2c1:
            rospy.logwarn("[mode3] failed to send 'down' over i2c; continuing")
        rospy.sleep(self.after_down_sleep_s)
        ok_i2c2 = self.ard2.send_text_retry("pillar_stop", retries=self.i2c_retries)
        if not ok_i2c2:
            rospy.logwarn("[mode3] failed to send 'pillar_stop' over i2c")

        # 5) ¿Ï·á ¾Ë¸²
        self.pub_mode_result.publish(String(data="reset"))
        rospy.loginfo("[mode3] published /mode_result = reset")

        # Á¤¸®
        self.ard1.close()
        self.ard2.close()

def main():
    rospy.init_node("mode3_reset_node")
    node = Mode3ResetNode()
    try:
        node.run_once()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
