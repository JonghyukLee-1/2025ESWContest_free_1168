#!/home/tunnel/jetson_project/yolov_env/bin/python
import time
import threading
import serial
import re
import Jetson.GPIO as GPIO

from smbus2 import SMBus
import rospy
from std_msgs.msg import String, Int32
from camera_path import capture_path_rtheta_validated

def P(name, default):
    return rospy.get_param("~" + name, default)

_NUM_RE = re.compile(r'^-?\d+(\.\d+)?$')

def _parse_distance(line: str):
    line = line.strip()
    if not line:
        return None
    token = line.split(",", 1)[1].strip() if line.startswith("DIST,") else line
    if not _NUM_RE.match(token):
        return None
    try:
        return float(token)
    except:
        return None

class Arduino1:
    def __init__(self, port="/dev/ttyACM0", baud=9600):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        time.sleep(2.0)

    def send_text(self, text: str):
        self.ser.write((text.strip() + "\n").encode())
        time.sleep(0.02)

    def send_move(self, motor_id: int, direction: str, speed: float, distance: float):
        self.ser.write(f"{motor_id},{direction},{speed:.2f},{distance:.2f}\n".encode())
        time.sleep(0.05)

    def close(self):
        try:
            self.ser.close()
        except:
            pass

class Arduino2I2C:
    def __init__(self, bus_id=0, addr=0x08):
        self.bus = SMBus(bus_id)
        self.addr = addr

    def _send(self, text: str):
        payload = bytearray(text.encode()[:31])
        length = len(payload)
        self.bus.write_i2c_block_data(self.addr, length, list(payload))
        time.sleep(0.02)

    def shoot(self, ms: int):
        self._send(f"shoot,{int(ms)}")

    def shoot_stop(self):
        self._send("shoot_stop")

    def close(self):
        try:
            self.bus.close()
        except:
            pass

class Mode2Node:
    _SEQ = [
        [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],
        [0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1],
    ]

    def __init__(self):
        rospy.init_node("mode2_node")
        self.i2c_bus_id   = P("i2c_bus", 0)
        self.i2c_addr     = P("i2c_address", 0x08)
        self.linear_speed = P("linear_speed_cm_s", 3.0)
        self.theta_scale  = P("theta_speed_scale", 3.0)
        self.segment_pad  = P("segment_settle_sec", 0.5)
        self.shoot_ms     = P("shoot_ms", 800)
        self.model_path   = P("model_path", "/home/tunnel/Desktop/riot/best.pt")
        self.cam_index    = P("camera_index", 0)
        self.conf_thr     = P("conf", 0.7)
        self.step_px      = P("step_px", 5.0)
        self.cx           = P("cx", 0.0)
        self.cy           = P("cy", 0.0)
        self.px2cm_x      = P("px2cm_x", 1.0)
        self.px2cm_y      = P("px2cm_y", 1.0)
        self.device       = P("device", "cuda:0")
        self.sort_from_90 = P("sort_from_90", True)
        self.dist_poll_period       = P("distance_poll_sec", 0.05)
        self.abnormal_timeout_s     = P("abnormal_timeout_sec", 12.0)
        self.abnormal_dist_threshold= P("abnormal_dist_threshold_cm", 10.0)
        self.max_ab_steps           = P("max_ab_steps", 3000)
        self.initial_r     = P("initial_r", 0.0)
        self.initial_theta = P("initial_theta", -90.0)
        self.pre_r         = P("pre_r", 0.0)
        self.pre_theta     = P("pre_theta", 90.0)
        self.post_r        = P("post_r", 0.0)
        self.post_theta    = P("post_theta", -90.0)
        self.IN1 = P("stepper_in1_bcm", 17)
        self.IN2 = P("stepper_in2_bcm", 18)
        self.IN3 = P("stepper_in3_bcm", 27)
        self.IN4 = P("stepper_in4_bcm", 22)
        self.cm_per_step = P("cm_per_step", 1.0/450.0)
        self.stepper_speed_cm_s = P("stepper_speed_cm_s", 2.0)
        self.pub_done     = rospy.Publisher("/mode_result", String, queue_size=1, latch=True)
        self.pub_ab_steps = rospy.Publisher("/steps", Int32,  queue_size=1, latch=True)
        self.a1 = Arduino1(self.serial_port, self.serial_baud)
        self.a2 = Arduino2I2C(self.i2c_bus_id, self.i2c_addr)
        self._ab_steps = 0
        self._ser_lock = threading.Lock()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in [self.IN1, self.IN2, self.IN3, self.IN4]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

    def safe_send_text(self, text: str, sleep_after=0.02):
        with self._ser_lock:
            self.a1.send_text(text)
        if sleep_after > 0:
            time.sleep(sleep_after)

    def safe_send_move(self, motor_id: int, direction: str, speed: float, distance: float, sleep_after=0.05):
        with self._ser_lock:
            self.a1.send_move(motor_id, direction, speed, distance)
        if sleep_after > 0:
            time.sleep(sleep_after)

    def _clear_input_buffer(self):
        with self._ser_lock:
            try:
                n = self.a1.ser.in_waiting
                if n:
                    self.a1.ser.read(n)
            except:
                pass

    def _coils_off(self):
        for pin in [self.IN1, self.IN2, self.IN3, self.IN4]:
            GPIO.output(pin, 0)

    def _speed_to_delay(self, speed_cm_s: float) -> float:
        steps_per_s = speed_cm_s / max(self.cm_per_step, 1e-9)
        return max(0.002, 1.0 / steps_per_s)

    def _step_once(self, direction: int, delay_s: float):
        seq = self._SEQ if direction >= 0 else list(reversed(self._SEQ))
        for pat in seq:
            GPIO.output(self.IN1, pat[0])
            GPIO.output(self.IN2, pat[1])
            GPIO.output(self.IN3, pat[2])
            GPIO.output(self.IN4, pat[3])
            time.sleep(delay_s)

    def _read_distance_locked(self, max_wait=None):
        if max_wait is None:
            max_wait = self.dist_poll_period
        t0 = time.time()
        with self._ser_lock:
            try:
                n = self.a1.ser.in_waiting
                if n:
                    self.a1.ser.read(n)
            except:
                pass
            while (time.time() - t0) < max_wait and not rospy.is_shutdown():
                try:
                    line = self.a1.ser.readline().decode('utf-8', errors='ignore')
                    if not line:
                        time.sleep(0.01)
                        continue
                    val = _parse_distance(line)
                    if val is not None:
                        return val
                except:
                    time.sleep(0.02)
                    break
        return None

    def _send_delta_move(self, dr, dth):
        if abs(dr) > 1e-6:
            self.safe_send_move(1, 'F' if dr >= 0 else 'B', self.linear_speed, abs(dr))
        if abs(dth) > 1e-6:
            self.safe_send_move(2, 'F' if dth >= 0 else 'B', self.linear_speed * self.theta_scale, abs(dth))

    def _estimate_move_time(self, abs_dr, abs_dth):
        t_r  = (abs_dr  / max(self.linear_speed, 1e-6)) if abs_dr  > 0 else 0.0
        t_th = (abs_dth / max(self.linear_speed * self.theta_scale, 1e-6)) if abs_dth > 0 else 0.0
        return max(t_r, t_th, 0.1)

    def _abnormal_move_until_distance_and_count(self):
        rospy.loginfo("[mode2] send 'abnormal' to Arduino1")
        self.safe_send_text("ABNORMAL", sleep_after=0.05)
        self._ab_steps = 0
        delay = self._speed_to_delay(self.stepper_speed_cm_s)
        thr = self.abnormal_dist_threshold
        deadline = time.time() + self.abnormal_timeout_s
        rospy.loginfo(f"[mode2] (local abnormal) backward until distance > {thr:.1f}cm")
        while not rospy.is_shutdown() and time.time() < deadline:
            self._step_once(direction=-1, delay_s=delay)
            self._ab_steps += 1
            val = self._read_distance_locked(max_wait=self.dist_poll_period)
            if val is not None and val > thr:
                rospy.loginfo(f"[mode2] distance {val:.2f} > {thr:.1f} ¡æ stop local abnormal")
                break
            if self._ab_steps >= self.max_ab_steps:
                rospy.logwarn("[mode2] max_ab_steps reached ¡æ stop local abnormal")
                break
        self._coils_off()
        self.safe_send_text("STOP", sleep_after=0.05)
        self._clear_input_buffer()

    def _publish_results_and_cleanup(self):
        self.pub_ab_steps.publish(Int32(self._ab_steps))
        rospy.loginfo(f"[mode2] published ab_steps={self._ab_steps} on {self.pub_ab_steps.resolved_name}")
        rospy.loginfo(f"[mode2] published 'done' on {self.pub_done.resolved_name}")
        self.safe_send_text("STOP", sleep_after=0.02)
        self.a2.shoot_stop()
        self.safe_send_text("QUIT", sleep_after=0.02)
        self.pub_done.publish("done")

    def run(self):
        try:
            self._abnormal_move_until_distance_and_count()
            dr0  = self.pre_r - self.initial_r
            dth0 = self.pre_theta - self.initial_theta
            rospy.loginfo(f"[mode2] pre-position to ({self.pre_r:.2f}, {self.pre_theta:.2f}) (¥Är={dr0:.2f}, ¥Ä¥è={dth0:.2f})")
            self._send_delta_move(dr0, dth0)
            time.sleep(self._estimate_move_time(abs(dr0), abs(dth0)) + self.segment_pad)
            rospy.loginfo("[mode2] capturing path (validated one-shot)...")
            path = capture_path_rtheta_validated(
                model_path=self.model_path,
                cam_index=self.cam_index,
                conf_thr=self.conf_thr,
                step_px=self.step_px,
                cx=(None if self.cx == 0.0 else self.cx),
                cy=(None if self.cy == 0.0 else self.cy),
                px2cm_x=self.px2cm_x,
                px2cm_y=self.px2cm_y,
                do_sort_from_90=self.sort_from_90,
                device=self.device,
                max_attempts=P("max_attempts", 10),
                timeout_s=P("capture_timeout_sec", 8.0),
                min_points=P("min_points", 16),
                debug_view=P("debug_view", False)
            )
            if not path:
                rospy.logwarn("[mode2] empty path. cleanup.")
                self._publish_results_and_cleanup()
                return
            prev_r, prev_th = self.pre_r, self.pre_theta
            for idx, (r, th) in enumerate(path, start=1):
                dr  = r  - prev_r
                dth = th - prev_th
                self._send_delta_move(dr, dth)
                move_time = self._estimate_move_time(abs(dr), abs(dth))
                time.sleep(move_time + self.segment_pad)
                rospy.loginfo(f"[mode2] shoot {self.shoot_ms}ms @ point {idx}/{len(path)}")
                self.a2.shoot(self.shoot_ms)
                time.sleep(min(self.shoot_ms/1000.0, 0.2))
                prev_r, prev_th = r, th
            drp  = self.post_r - prev_r
            dthp = self.post_theta - prev_th
            rospy.loginfo(f"[mode2] post-position to ({self.post_r:.2f}, {self.post_theta:.2f}) (¥Är={drp:.2f}, ¥Ä¥è={dthp:.2f})")
            self._send_delta_move(drp, dthp)
            time.sleep(self._estimate_move_time(abs(drp), abs(dthp)) + self.segment_pad)
            self._publish_results_and_cleanup()
            rospy.loginfo("[mode2] finished. Ctrl+C to exit.")
            rate = rospy.Rate(5)
            while not rospy.is_shutdown():
                rate.sleep()
        finally:
            try:
                GPIO.cleanup()
            except:
                pass
            self.a2.close()
            self.a1.close()

def main():
    node = Mode2Node()
    node.run()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
