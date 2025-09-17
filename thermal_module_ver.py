
#!/home/tunnel/jetson_project/yolov_env/bin/python

import os
os.environ["BLINKA_I2C_BUS"] = "1"

import time, board, busio
import numpy as np
import adafruit_mlx90640
import datetime as dt
import cv2
import rospy
from std_msgs.msg import Float32, Int32, Float32MultiArray, Bool

SHAPE = (24, 32)
RESIZED = (640, 480)
threshold = 1.5
AREA_MIN = 10
scale_x = RESIZED[0] / SHAPE[1]
scale_y = RESIZED[1] / SHAPE[0]

def mlx_setting():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)
    
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ
    return mlx

def temp_to_u8(arr, vmin=26.0, vmax=36.0):
    clipped = np.clip(arr, vmin, vmax)
    return ((clipped - vmin) * (255.0 / (vmax - vmin))).astype(np.uint8 ).reshape(SHAPE)

def compute_gradient_mask(img_f32, threshold):
    gy, gx = np.gradient(img_f32)
    grad_mag = np.sqrt(gx**2 + gy**2)
    mask = (grad_mag > threshold).astype(np.uint8)
    return mask

def clean_mask(mask):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    return mask

def find_hot_centers(mask, stats_area_min=AREA_MIN):
    centers = []
    n_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for i in range(1, n_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < stats_area_min:
            continue
        cx, cy = centroids[i]
        px = int(cx * scale_x)
        py = int(cy * scale_y)
        centers.append((px, py))
    return centers

def draw_centers(vis, centers):
    for (px, py) in centers:
        cv2.circle(vis, (px, py), 8, (0, 255, 255), -1)

# not actually center. how much pixel do we have to give? 
def about_center(centers):
    if not centers:
        return False, None, None, None, None, None, None
    else:
        if_centers = True 
    rightmost = max(centers, key=lambda p: p[0])
    rightmost_x = rightmost[0]
    rightmost_y = rightmost[1]
    move_x = (rightmost_x - 320) 
    if move_x > 5: # right
        have_x = 1
    elif move_x < -5: # left
        have_x = -1
    elif -5 <= move_x <=5 : # stop. how much we should give? 
        have_x = 0
    move_y = (rightmost_y - 240)
    if move_y < 0: # up
        have_y = 1
    elif move_y > 0: # down
        have_y = -1
    elif move_y == 0: #stop
        have_y = 0
    return if_centers, move_x, move_y, rightmost_x, rightmost_y, have_x, have_y


# absolutely center_ver
"""
def about_center(centers):
    if not centers:
        return False, None, None, None, None, None, None
    else:
        if_centers = True 
    rightmost = max(centers, key=lambda p: p[0])
    rightmost_x = rightmost[0]
    rightmost_y = rightmost[1]
    move_x = (rightmost_x - 320) 
    if move_x > 0: # right
        have_x = 1
    elif move_x < 0: # left
        have_x = -1
    elif move_x == 0: # stop. how much we should give? 
        have_x = 0
    move_y = (rightmost_y - 240)
    if move_y < 0: # up
        have_y = 1
    elif move_y > 0: # down
        have_y = -1
    elif move_y == 0: #stop
        have_y = 0
    return if_centers, move_x, move_y, rightmost_x, rightmost_y, have_x, have_y
"""

def show_info(vis, frame, fps):
    info = f"Tmin={frame.min():+.1f}  Tmax={frame.max():+.1f}  FPS={fps:.2f}"
    cv2.putText(vis, info, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

def main():
    rospy.init_node('thermal_node')
    pub_ifcenters = rospy.Publisher('/thermal_flag_mode0', Bool, queue_size=1)
    pub_movex = rospy.Publisher('/move_x', Int32, queue_size=1)
    pub_movey = rospy.Publisher('/move_y', Int32, queue_size=1)
    pub_rightmostx = rospy.Publisher('/thermal_rightmostx', Int32, queue_size=1)
    pub_rightmosty = rospy.Publisher('/thermal_rightmosty', Int32, queue_size=1)
    pub_havex = rospy.Publisher('/have_to_move_x', Int32, queue_size=1)
    pub_havey = rospy.Publisher('/have_to_move_y', Int32, queue_size=1)
    
    mlx = mlx_setting()
    frame = np.zeros(24 * 32, dtype=np.float32)
    print("code start")
    time.sleep(4)

    try:
        while True:
            t0 = time.time()

            try:
                mlx.getFrame(frame)
            except ValueError as e:
                rospy.logwarn(f"[THERMAL] getFrame skipped: {e}")
                continue  # skip this frame and go next loop

            # extra safety: drop frames containing NaN/Inf
            if not np.isfinite(frame).all():
                rospy.logwarn("[THERMAL] invalid frame skipped")
                continue

            img_f32 = frame.reshape(SHAPE)
            u8_img = temp_to_u8(img_f32)
            vis = cv2.applyColorMap(u8_img, cv2.COLORMAP_JET)
            vis = cv2.resize(vis, RESIZED, cv2.INTER_CUBIC)

            mask = compute_gradient_mask(img_f32, threshold)
            mask = clean_mask(mask)
            centers = find_hot_centers(mask)

            if_centers,move_x, move_y, rightmost_x, rightmost_y, have_x, have_y = about_center(centers)

            pub_ifcenters.publish(Bool(data=if_centers))

            if if_centers is not False:
                pub_movex.publish(Int32(data=move_x))
                pub_movey.publish(Int32(data=move_y))
                pub_rightmostx.publish(Int32(data=rightmost_x))
                pub_rightmosty.publish(Int32(data=rightmost_y))
                pub_havex.publish(Int32(data=have_x))
                pub_havey.publish(Int32(data=have_y))

            draw_centers(vis, centers)
            # determine_move(centers)

            fps = 1.0 / (time.time() - t0)
            show_info(vis, frame, fps)
            cv2.imshow("Output", vis)
            # cv2.waitKey(1)

            #press 'q' to quit the window
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break


    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Stopped")

if __name__ == "__main__":
    main()
