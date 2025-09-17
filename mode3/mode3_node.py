#!/home/tunnel/jetson_project/yolov_env/bin/python
# we have to move arm back and get pillar down here.
# we can control arm by steps, but how about pillar? --> we have to get motor data from encoder.
# pillar_modiifed_ver(just)
import time
import threading
import serial
import rospy
from std_msgs.msg import String, Int32, Bool, Float32

from arm_control import cleanup_gpio, move_motor


def P(name, default):
    return rospy.get_param("~" + name, default)

class ArduinoSerial:

    
    """Generic UART helper"""
    def __init__(self, port, baud=9600, timeout=1.0, label="ARD"):
        self.port, self.baud, self.timeout, self.label = port, baud, timeout, label
        self.ser = None


    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2.0)
        rospy.loginfo(f"[mode3] Serial opened ({self.label}): {self.port} @ {self.baud}")

    def send_line_retry(self, line: str, retries=3, backoff=0.2):
        msg = (line.strip() + "\n").encode("utf-8")
        for i in range(retries):
            try:
                self.ser.write(msg)
                self.ser.flush()
                rospy.loginfo(f"[mode3] -> {self.label}: {line.strip()} (try {i+1})")
                return True
            except Exception as e:
                rospy.logwarn(f"[mode3] {self.label} send failed({i+1}/{retries}): {e}")
                time.sleep(backoff * (i+1))
        return False

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except:
            pass

class Mode3ResetNode:
    def __init__(self):
        # ==== Params ====
        self.ard1_port = P("ard1_port", "/dev/ttyACM0")   # Arduino1
        self.ard2_port = P("ard2_port", "/dev/ttyACM1")   # Arduino2
        self.serial_baud = P("serial_baud", 9600)
        self.ser_retries = int(P("serial_retries", 3))

        #self.move_done_timeout  = float(P("move_done_timeout", 20.0))
        self.after_down_sleep_s = float(P("after_down_sleep_s", 3.0))

        self.remain_steps_topic = P("remain_steps_topic", "/remain_steps")
        self.move_cmd_topic     = P("move_cmd_topic", "/mode3/move_steps")
        #self.move_done_topic    = P("move_done_topic", "/mode3/move_done")
        self.mode_result_topic  = P("mode_result_topic", "/mode_result")

        # ==== State ====
        self._lock = threading.Lock()
        self._remain_steps = None
        self._consumed_steps = False
        #self._move_done = False
        self._ran = False
        self.time_for_pillar_down = None
        

        # ==== ROS IO ====
        self.pub_move_cmd = rospy.Publisher(self.move_cmd_topic, Int32, queue_size=1, latch=True)
        self.pub_mode_result = rospy.Publisher(self.mode_result_topic, String, queue_size=1, latch=True)
        rospy.Subscriber("/pillar_down_time", Float32, self._cb_for_pillar_down)
        rospy.Subscriber(self.remain_steps_topic, Int32, self._cb_remain_steps)
        #rospy.Subscriber(self.move_done_topic, Bool, self._cb_move_done)

        # ==== HW (UART only) ====
        self.ard1 = ArduinoSerial(self.ard1_port, self.serial_baud, label="Arduino1")
        self.ard2 = ArduinoSerial(self.ard2_port, self.serial_baud, label="Arduino2")

    # ---------- Callbacks ----------
    def _cb_remain_steps(self, msg: Int32):
        with self._lock:
            if self._consumed_steps:
                return
            self._remain_steps = int(msg.data)
            rospy.loginfo(f"[mode3] received /remain_steps = {self._remain_steps} (latched)")

    # def _cb_move_done(self, msg: Bool):
    #     if msg.data:
    #         with self._lock:
    #             self._move_done = True
    #         rospy.loginfo("[mode3] move_done TRUE")

    def _cb_for_pillar_down(self, msg: Float32):
        self.time_for_pillar_down = float(msg.data)
        rospy.loginfo(f"[mode3] received pillar_time = {self.time_for_pillar_down:.1f} sec")


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

    def _normal_mode_move(self, steps: int, direction=1):
        rospy.loginfo(f"[mode3] normal_mode: have to move_steps={steps}")
        try:
            move_motor(steps,delay=0.003, direction = 1)
            #self.pub_move_done.publish(Bool(data=True))
            #rospy.loginfo("[mode3] Arm movement done, published move_done=True")
            return True
        
        except Exception as e:
            rospy.logwarn(f"[mode3] arm move failed: {e}")
            return False
        finally:
            cleanup_gpio()

    # ---------- Flow ----------
    def run_once(self):
        try:
            with self._lock:
                if self._ran:
                    rospy.logwarn("[mode3] already executed; ignoring re-entry")
                    return
                self._ran = True

            try:
                self.ard1.open()
                self.ard2.open()
            except Exception as e:
                rospy.logerr(f"[mode3] serial open failed: {e}")
                return


            # 1) steps latch
            steps = self._wait_first_steps(timeout=15.0)
            direction = 1 if steps >= 0 else -1   # ?쩍쩌철쨍챕 쨔횦쨈챘쨔챈횉창
            steps = abs(steps)

            # 2) Arduino1 (UART1): normal
            self.ard1.send_line_retry("normal", retries=self.ser_retries) # why do we have this?

            # 3) normal_mode move
            self._normal_mode_move(steps, direction)

            # 4) Arduino2 (UART2): down -> pillar_stop
            self.ard2.send_line_retry("down", retries=self.ser_retries)
            if self.time_for_pillar_down is not None:
                wait_s = self.time_for_pillar_down
            else:
                wait_s = self.after_down_sleep_s # basic setting.
            rospy.sleep(wait_s)
            self.ard2.send_line_retry("pillar_stop", retries=self.ser_retries)

            # 5) publish reset
            self.pub_mode_result.publish(String(data="reset")) # cord is really finished.
            rospy.loginfo("[mode3] published /mode_result = reset")

            # cleanup
        finally:
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
