#!/home/tunnel/jetson_project/yolov_env/bin/python
# -*- coding: utf-8 -*-

"""
Detect a 'tile' polygon with YOLOv8 in camera frames, extract its outer contour,
resample the contour into evenly spaced points, convert (x, y) to (r, rhteta) in
robot-centric coordinates, then sort angles starting at 90징횈 and going
clockwise down to -270 degree. Finally, return the path as an ordered list.

- Prereqs: torch + CUDA if available (set device="cuda:0"), otherwise use "cpu".
"""

import time
import math
import cv2
import numpy as np
from ultralytics import YOLO


# =========================== Geometry helpers =========================== #

def _dist(a, b):
    """Return Euclidean distance between points a=(x1, y1), b=(x2, y2)."""
    return float(np.hypot(b[0] - a[0], b[1] - a[1]))


def _cumsum_lengths(pts):
    """
    Compute cumulative chord lengths along a polyline.
    For [p0, p1, p2, ...] return [0, |p0p1|, |p0p1|+|p1p2|, ...].
    """
    if len(pts) == 0:
        return []
    acc = [0.0]
    total = 0.0
    for i in range(1, len(pts)):
        total += _dist(pts[i - 1], pts[i])
        acc.append(total)
    return acc


def resample_polyline_xy(points_xy, step_px=5.0, include_last=True):
    """
    Resample a polyline to equal arc-length spacing.
    - Takes a list of (x, y) points and produces points at spacing step_px.
    - The polyline is traversed in-order; intermediate points are linearly interpolated.
    """
    n = len(points_xy)
    if n <= 1:
        return points_xy[:]
    lengths = _cumsum_lengths(points_xy)
    total = lengths[-1]
    if total == 0.0:
        return [points_xy[0]]

    # Targets along the arclength parameter (0, step, 2*step, ..., total)
    targets = [i * step_px for i in range(int(total // step_px) + 1)]
    if include_last and targets[-1] < total:
        targets.append(total)

    out = []
    j = 0
    for L in targets:
        while j < n - 1 and lengths[j + 1] < L:
            j += 1
        if j >= n - 1:
            out.append(points_xy[-1])
        else:
            L0, L1 = lengths[j], lengths[j + 1]
            denom = (L1 - L0)
            t = 0.0 if denom <= 1e-9 else (L - L0) / denom
            x = points_xy[j][0] + (points_xy[j + 1][0] - points_xy[j][0]) * t
            y = points_xy[j][1] + (points_xy[j + 1][1] - points_xy[j][1]) * t
            out.append((x, y))
    return out


def xy_to_rtheta_robot(points_xy, cx, cy, px2cm_x=1.0, px2cm_y=1.0):
    """
    Convert image-space (x, y) points into robot-centric polar (r, 짜챔).
    - (cx, cy) is the robot's reference point in the image frame.
    - px2cm_x, px2cm_y are pixel징챈cm scale factors per axis.
    - Angle convention: 짜챔_robot = 90징횈 - atan2(dy, dx)
      Then wrap to the interval (-270징횈, 90징횈], i.e., starting at 90징횈 and
      going clockwise down to -270징횈.
    """
    out = []
    for x, y in points_xy:
        dx = (x - cx) * px2cm_x
        dy = (y - cy) * px2cm_y
        r = float(np.hypot(dx, dy))
        theta_math = float(np.degrees(np.arctan2(dy, dx)))
        theta_robot = 90.0 - theta_math
        # Wrap angle into (-270, 90]
        while theta_robot > 90.0:
            theta_robot -= 360.0
        while theta_robot <= -270.0:
            theta_robot += 360.0
        out.append((r, theta_robot))
    return out


def sort_rtheta_from_90_clockwise(rt_list):
    """
    Sort (r, 짜챔) so angles start at 90징횈 and proceed clockwise to -270징횈.
    - Compute delta = 짜챔 - 90 and wrap delta into (-360, 0].
    - Sort by delta descending, so 0 (i.e., 90징횈) comes first and -360 last.
    """
    buf = []
    for r, th in rt_list:
        delta = th - 90.0
        # Wrap delta to (-360, 0]
        while delta > 0:
            delta -= 360.0
        while delta <= -360.0:
            delta += 360.0
        buf.append((r, th, delta))
    # Sort by wrapped delta descending: 0 징챈 -360
    buf.sort(key=lambda x: x[2], reverse=True)
    return [(r, th) for r, th, _ in buf]


# =========================== Main path capture logic =========================== #

def capture_path_rtheta(
    model_path,
    cam_index=0,
    conf_thr=0.7,
    step_px=8.0,
    cx=None, cy=None,
    px2cm_x=1.0, px2cm_y=1.0,
    device="cuda:0",
    timeout_open_s=10.0,
    max_attempts=10,
    timeout_detect_s=8.0,
    min_points=12,
    debug_view=True
):
    """
    Pipeline:
    - Open a camera stream (with timeout).
    - Run YOLO to segment class 'tile'.
    - Find the contour that contains the image center (robot reference).
    - Resample its boundary with spacing step_px.
    - Convert the resampled (x, y) into (r, 짜챔) in robot coordinates.
    - Sort by angle from 90징횈 clockwise to -270징횈 and return that ordered path.
    """
    model = YOLO(model_path)

    # --- Open camera with retry until timeout_open_s --- #
    t_deadline = time.time() + timeout_open_s
    cap = None
    while time.time() < t_deadline:
        cap = cv2.VideoCapture(cam_index)
        if cap.isOpened():
            break
        time.sleep(0.2)
    if cap is None or not cap.isOpened():
        raise RuntimeError(f"Camera open timeout: index={cam_index}")

    # --- Warmup reads to stabilize exposure/first frames --- #
    for _ in range(3):
        cap.read()
        time.sleep(0.02)

    t_start = time.time()
    attempts = 0
    chosen_path = []

    while attempts < max_attempts and (time.time() - t_start) < timeout_detect_s:
        attempts += 1
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        _cx = cx if cx is not None else (w / 2.0)
        _cy = cy if cy is not None else (h / 2.0)
        center = (_cx, _cy)

        # --- YOLOv8 inference --- #
        results = model(frame, conf=conf_thr, device=device)[0]

        vis = None
        overlay = None
        if debug_view:
            vis = frame.copy()
            overlay = np.zeros_like(frame)

        found = False
        rt_sorted = []

        # --- Use the segmentation mask outputs if present --- #
        if results.masks is not None:
            # Iterate over masks and corresponding class IDs
            for mask_t, cls in zip(results.masks.data, results.boxes.cls):
                name = results.names[int(cls.item())]
                if name != "tile":
                    continue

                # Convert mask tensor to 0/255 uint8 image
                mask = (mask_t.detach().cpu().numpy().astype(np.uint8) * 255)

                # Filter by mask area (remove tiny or full-frame masks)
                area = int(np.sum(mask > 0))
                if area < 300 or area > w * h * 0.9:
                    continue

                # Slightly dilate to consolidate edges
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                mask = cv2.dilate(mask, kernel, iterations=1)

                # Extract external contours from the mask
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                for cnt in cnts:
                    if cv2.contourArea(cnt) < 300:
                        continue

                    # Choose the contour that contains the reference center point
                    if cv2.pointPolygonTest(cnt, center, False) >= 0:
                        # Convert contour to (N, 2) float
                        contour_xy = cnt.reshape(-1, 2).astype(np.float32)

                        # Ensure the polyline is closed to avoid a seam
                        if not (contour_xy[0] == contour_xy[-1]).all():
                            contour_xy = np.vstack([contour_xy, contour_xy[0]])

                        # Resample the contour into equal arc-length points
                        resampled_xy = resample_polyline_xy(
                            contour_xy.tolist(),
                            step_px=step_px,
                            include_last=True
                        )

                        # Convert to (r, 짜챔) in robot coordinates
                        rt = xy_to_rtheta_robot(
                            resampled_xy, cx=_cx, cy=_cy,
                            px2cm_x=px2cm_x, px2cm_y=px2cm_y
                        )

                        # Discard if too few points after resampling
                        if len(rt) < min_points:
                            continue

                        # Sort by angle: start at 90징횈 징챈 clockwise 징챈 -270징횈
                        rt_sorted = sort_rtheta_from_90_clockwise(rt)
                        found = True

                        # Optional visualization overlay
                        if debug_view:
                            overlay[mask == 255] = (0, 255, 0)  # mask overlay
                            vis = cv2.addWeighted(vis, 1.0, overlay, 0.3, 0)
                            cv2.drawContours(vis, [cnt], -1, (0, 0, 255), 2)  # contour
                            cv2.circle(vis, (int(_cx), int(_cy)), 5, (255, 0, 255), -1)  # center

                        break  # We only need the contour that contains the center
                if found:
                    break

        # If you want live preview instead of blocking viewer, use the snippet below:
        # if debug_view:
        #     cv2.imshow("tile_capture", vis if vis is not None else frame)
        #     cv2.waitKey(1)

        if found and rt_sorted:
            chosen_path = rt_sorted
            break
    
    if debug_view and found and vis is not None:
        cv2.imshow("tile_capture", vis)
        print("Press q or ESC to close the window...")
        while True:
            k = cv2.waitKey(0) & 0xFF
            if k in (27, ord('q')):
                break

    cap.release()
    if debug_view:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    return chosen_path

'''
# =========================== Example (manual run) =========================== #

if __name__ == "__main__":
    # -- Model path (update as needed) -- #
    model_path = "/home/tunnel/Desktop/riot/best.pt"

    # -- Capture -- #
    path = capture_path_rtheta(
        model_path=model_path,
        cam_index=0,
        conf_thr=0.7,
        step_px=8.0,          # resampling step in pixels
        cx=None, cy=None,     # None 징챈 use image center as robot reference
        px2cm_x=0.1,          # pixel-to-cm scale factors (tune for your setup)
        px2cm_y=0.1,
        device="cpu",         # or "cuda:0" for GPU
        timeout_open_s=10.0,
        max_attempts=10,
        timeout_detect_s=8.0,
        min_points=12,
        debug_view=True
    )

    # -- Print results -- #
    if not path:
        print("No path detected.")
    else:
        print("Captured (r, theta) path (sorted 90징횈 CW to -270징횈):")
        for i, (r, th) in enumerate(path[:60]):  # print first 60 for brevity
            print(f"{i:03d}: r={r:.2f}, theta={th:.1f}")
'''
