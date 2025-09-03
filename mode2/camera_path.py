#!/home/tunnel/jetson_project/yolov_env/bin/python
import time
import cv2
import numpy as np
from ultralytics import YOLO

def _dist(a, b):
    return float(np.hypot(b[0] - a[0], b[1] - a[1]))

def _cumsum_lengths(pts):
    if len(pts) == 0:
        return []
    acc = [0.0]
    total = 0.0
    for i in range(1, len(pts)):
        total += _dist(pts[i-1], pts[i])
        acc.append(total)
    return acc

def resample_polyline_xy(points_xy, step_px=5.0, include_last=True):
    n = len(points_xy)
    if n <= 1:
        return points_xy[:]
    lengths = _cumsum_lengths(points_xy)
    total = lengths[-1]
    if total == 0.0:
        return [points_xy[0]]
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
            x = points_xy[j][0] + (points_xy[j+1][0] - points_xy[j][0]) * t
            y = points_xy[j][1] + (points_xy[j+1][1] - points_xy[j][1]) * t
            out.append((x, y))
    return out

def sort_rtheta_clockwise_from_90(rt_list):
    buf = []
    for r, th in rt_list:
        delta = th - 90.0
        while delta > 0:
            delta -= 360.0
        while delta <= -360.0:
            delta += 360.0
        buf.append((r, th, delta))
    buf.sort(key=lambda x: x[2], reverse=True)
    return [(r, th) for r, th, _ in buf]

def xy_to_rtheta_robot(points_xy, cx, cy, px2cm_x=1.0, px2cm_y=1.0):
    out = []
    for x, y in points_xy:
        dx = (x - cx) * px2cm_x
        dy = (y - cy) * px2cm_y
        r = float(np.hypot(dx, dy))
        theta_math = float(np.degrees(np.arctan2(dy, dx)))
        theta_robot = 90.0 - theta_math
        while theta_robot <= -180.0:
            theta_robot += 360.0
        while theta_robot > 180.0:
            theta_robot -= 360.0
        out.append((r, theta_robot))
    return out

def capture_path_rtheta_validated(model_path,
                                  cam_index=0,
                                  conf_thr=0.7,
                                  step_px=5.0,
                                  cx=None, cy=None,
                                  px2cm_x=1.0, px2cm_y=1.0,
                                  do_sort_from_90=True,
                                  device="cuda:0",
                                  max_attempts=10,
                                  timeout_s=8.0,
                                  min_points=16,
                                  debug_view=False):
    model = YOLO(model_path)
    open_deadline = time.time() + 5.0
    cap = None
    while time.time() < open_deadline:
        cap = cv2.VideoCapture(cam_index)
        if cap.isOpened():
            break
        time.sleep(0.2)
    if cap is None or not cap.isOpened():
        raise RuntimeError(f"Camera open timeout: index={cam_index}")

    for _ in range(3):
        cap.read()
        time.sleep(0.02)

    start = time.time()
    attempt = 0
    chosen_path = []

    while attempt < max_attempts and (time.time() - start) < timeout_s:
        attempt += 1
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        _cx = cx if cx is not None else (w / 2.0)
        _cy = cy if cy is not None else (h / 2.0)
        center = (_cx, _cy)

        results = model(frame, conf=conf_thr, device=device)[0]

        vis = None
        if debug_view:
            vis = frame.copy()
            overlay = np.zeros_like(frame)

        found = False
        candidate_rt = []

        if results.masks is not None:
            for mask_t, cls in zip(results.masks.data, results.boxes.cls):
                name = results.names[int(cls.item())]
                if name != "tile":
                    continue

                mask = (mask_t.detach().cpu().numpy().astype(np.uint8) * 255)
                area = int(np.sum(mask > 0))
                if area < 300 or area > w * h * 0.8:
                    continue

                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                mask = cv2.dilate(mask, kernel, iterations=1)

                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in cnts:
                    if cv2.contourArea(cnt) < 300:
                        continue
                    if cv2.pointPolygonTest(cnt, center, False) >= 0:
                        contour_xy = cnt.reshape(-1, 2).astype(float)
                        resampled_xy = resample_polyline_xy(contour_xy.tolist(), step_px=step_px, include_last=True)
                        rt = xy_to_rtheta_robot(resampled_xy, cx=_cx, cy=_cy, px2cm_x=px2cm_x, px2cm_y=px2cm_y)
                        if len(rt) < min_points:
                            continue
                        candidate_rt = sort_rtheta_clockwise_from_90(rt) if do_sort_from_90 else rt
                        found = True
                        if debug_view:
                            cv2.drawContours(vis, [cnt], -1, (0, 0, 255), 2)
                            overlay[mask == 255] = (0, 255, 0)
                            vis = cv2.addWeighted(vis, 1.0, overlay, 0.3, 0)
                            cv2.circle(vis, (int(_cx), int(_cy)), 5, (255, 0, 255), -1)
                        break
                if found:
                    break

        if debug_view:
            cv2.imshow("capture_check", vis if vis is not None else frame)
            cv2.waitKey(1)

        if found and candidate_rt:
            chosen_path = candidate_rt
            break

    cap.release()
    if debug_view:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    return chosen_path
