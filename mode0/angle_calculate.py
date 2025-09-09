# angle_calculate.py
import serial, time, re, math
import rospy

_NUM_RE = re.compile(r'^-?\d+(\.\d+)?$')

def _wait_ready_lines(ser, timeout=2.0):
    t0, buf = time.time(), b""
    while time.time() - t0 < timeout:
        try:
            chunk = ser.read(64)
        except Exception:
            break
        if chunk:
            buf += chunk
            if (b'READY' in buf) or (b'Ready' in buf) or (b'DIST,' in buf):
                return True
        time.sleep(0.02)
    return False

def open_arduino(port='/dev/ttyACM0', baud=9600, timeout=0.2):
    """Opens serial connection to Arduino and waits for it to be ready."""
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(2.0)  # Wait for auto-reset
    try:
        ser.reset_input_buffer(); ser.reset_output_buffer()
    except Exception:
        pass
    _wait_ready_lines(ser, timeout=2.0)
    return ser

def close_arduino(ser):
    """Closes serial connection safely."""
    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass

def _drain(ser):
    """Drains any waiting data from the serial buffer."""
    if hasattr(ser, "in_waiting"):
        try:
            n = ser.in_waiting
            if n: ser.read(n)
        except Exception:
            pass

def _parse_distance(line: str):
    if not line: return None
    line = line.strip()
    token = line.split(",", 1)[1].strip() if line[:5].upper() == "DIST," else line
    if not _NUM_RE.match(token): return None
    try: return float(token)
    except Exception: return None

def listen_from_arduino(ser, max_wait=1.0):
    """Listens for a single distance measurement (cm) from the serial port."""
    if ser is None: return None
    _drain(ser)
    t0 = time.time()
    while time.time() - t0 < max_wait and not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            line = ""
        if not line:
            time.sleep(0.01); continue
        val = _parse_distance(line)
        if val is not None:
            rospy.loginfo(val)
            return val
    return None

def get_angle(distance_cm: float, pillar_height: float = 63.0,
             min_deg: int = 0, max_deg: int = 180) -> int:
    """Calculates servo angle (deg): atan2(h, d), clamped to [min,max]."""
    if distance_cm is None or distance_cm <= 0:
        return max_deg
    ang = int(round(math.degrees(math.atan2(float(pillar_height), float(distance_cm)))))
    return max(min(ang, max_deg), min_deg)
