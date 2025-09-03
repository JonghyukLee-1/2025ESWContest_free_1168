#!/home/tunnel/jetson_project/yolov_env/bin/python

# common_i2c_lock.py
import fcntl, os, contextlib, time

_LOCK_PATH = "/tmp/i2c-1.lock"

@contextlib.contextmanager
def i2c_lock(block=True, timeout=0.03, poll=0.003):
    """
    Cross-process I2C bus lock using flock.
    block=True  : wait until lock acquired.
    block=False : try until timeout (non-blocking style).
    """
    os.makedirs("/tmp", exist_ok=True)
    with open(_LOCK_PATH, "w") as f:
        if block:
            fcntl.flock(f, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(f, fcntl.LOCK_UN)
        else:
            t0 = time.time()
            while True:
                try:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.time() - t0 >= timeout:
                        raise
                    time.sleep(poll)
            try:
                yield
            finally:
                fcntl.flock(f, fcntl.LOCK_UN)
