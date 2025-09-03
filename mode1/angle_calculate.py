import serial, time, numpy as np, re
import rospy


pillar_height = 63.0
distance_cm = 999.0
_arduino = None

_READY_PATTERNS = (b'READY', b'Ready', b'DIST,')
_NUM_RE = re.compile(r'^-?\d+(\.\d+)?$')

def _wait_ready_lines(ser, timeout=2.0):
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if any(p in buf for p in _READY_PATTERNS):
                return True
        time.sleep(0.02)
    return False

def open_arduino(port='/dev/ttyACM0', baud=9600, timeout=0.2):
    global _arduino
    if _arduino is None or not _arduino.is_open:
        _arduino = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)
        _arduino.reset_input_buffer()
        _arduino.reset_output_buffer()
        _wait_ready_lines(_arduino, timeout=2.0)
    return _arduino

def close_arduino():
    global _arduino
    if _arduino and _arduino.is_open:
        _arduino.close()
    _arduino = None

def _parse_distance(line: str):
    line = line.strip()
    if not line:
        return None
    if line[:5].upper() == "DIST,":
        token=line.split(",",1)[1].strip()
    else:
        token = line
    #token = line.split(",", 1)[1].strip() if line.startswith("DIST,") else line
    if not _NUM_RE.match(token):
        return None
    try:
        return float(token)
    except Exception:
        return None

def listen_from_arduino(max_wait=1.0, ser=None, port='/dev/ttyACM0', baud=9600, timeout=0.2):
    global distance_cm
    s = ser if ser is not None else open_arduino(port, baud, timeout)
    # drain current buffer so we only read fresh data
    try:
        n = s.in_waiting
        if n:
            s.read(n)
    except:
        pass

    t0 = time.time()
    while (time.time() - t0) < max_wait:
        try:
            line = s.readline().decode('utf-8', errors='ignore')
            if not line:
                time.sleep(0.01)
                continue
            val = _parse_distance(line)
            if val is not None:
                distance_cm = val
                rospy.loginfo(distance_cm)
                return distance_cm
        except Exception as e:
            rospy.logwarn(f"serial read error : {e}")
            time.sleep(0.05)
            break
    return None

def get_angle(d_cm):
    """Convert distance(cm) to servo angle (0~180)"""
    try:
        d = float(d_cm)
        if d <= 0:
            return None
        angle_rad = np.arctan(pillar_height / d)
        angle_deg = np.degrees(angle_rad)
        if ( 0.0 <= angle_deg <=180.0 ):
            return angle_deg
        else:
            return None
    except:
        return None

def angle_to_pwm(angle_deg, min_pwm=0, max_pwm=255):
    """Map servo angle (0~180) to PWM value (0~255)"""
    if angle_deg is None:
        return None
    angle_deg = max(0, min(180, angle_deg))
    pwm_val = int(min_pwm + (angle_deg / 180.0) * (max_pwm - min_pwm))
    return pwm_val
