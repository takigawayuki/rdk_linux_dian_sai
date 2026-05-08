# -*- coding: utf-8 -*-
"""
激光点映射导航（激光补偿版）

思路：
  1. CenterGet 检测靶板外框，拿中心坐标 + 轮廓面积
  2. 面积 → 距离：D = D_ref × sqrt(A_ref / A)
  3. 距离 → 激光点坐标：X 固定，Y 在 [Y_min, Y_max] 线性插值
  4. 串口发送 (dx, dy) = 激光点 - 靶心，让云台把激光点拉到靶心上

与原版 my_send2gimbal.py 区别：
  - 原版误差 = 靶心 - 画面中心       （让摄像头准心对准靶心）
  - 本版误差 = 激光点 - 靶心          （让激光打在靶心上，更符合实际打靶需求）
"""
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import cv2
import time
import threading
import queue
import numpy as np

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet

# ============================================================
# 串口 / 帧率
# ============================================================
SERIAL_PORT   = "/dev/ttyS1"
BAUDRATE      = 115200
SEND_INTERVAL = 0.01        # 100Hz

# ============================================================
# 标定参数（从 laser_calib_params.json 自动加载，没有就用默认）
# ============================================================
REFERENCE_AREA_PIXELS = 7461    # 靶板在参考距离处的像素面积
REFERENCE_DISTANCE_MM = 1350    # 参考标定距离
MIN_DISTANCE_MM       = 790     # 有效测距下限
MAX_DISTANCE_MM       = 1350    # 有效测距上限
LASER_X_FIXED         = 331     # 激光点 X 固定值
LASER_Y_MIN_DISTANCE  = 215     # MIN_DISTANCE_MM 处激光 Y
LASER_Y_MAX_DISTANCE  = 222     # MAX_DISTANCE_MM 处激光 Y

PARAM_FILE = os.path.join(os.path.dirname(__file__), "laser_calib_params.json")
if os.path.exists(PARAM_FILE):
    with open(PARAM_FILE, "r", encoding="utf-8") as f:
        _p = json.load(f)
    REFERENCE_AREA_PIXELS = _p.get("REFERENCE_AREA_PIXELS", REFERENCE_AREA_PIXELS)
    LASER_X_FIXED         = _p.get("LASER_X_FIXED",         LASER_X_FIXED)
    LASER_Y_MIN_DISTANCE  = _p.get("LASER_Y_MIN_DISTANCE",  LASER_Y_MIN_DISTANCE)
    LASER_Y_MAX_DISTANCE  = _p.get("LASER_Y_MAX_DISTANCE",  LASER_Y_MAX_DISTANCE)
    print(f"[参数] 已从 {os.path.basename(PARAM_FILE)} 加载")

print(f"[参数] REF: A={REFERENCE_AREA_PIXELS}px @ D={REFERENCE_DISTANCE_MM}mm")
print(f"[参数] 距离范围: {MIN_DISTANCE_MM}~{MAX_DISTANCE_MM}mm")
print(f"[参数] 激光: X={LASER_X_FIXED}, Y={LASER_Y_MIN_DISTANCE}~{LASER_Y_MAX_DISTANCE}")

# ============================================================
# 队列 / 共享状态
# ============================================================
frame_queue  = queue.Queue(maxsize=2)   # 采集 → 算法
frame_queue2 = queue.Queue(maxsize=2)   # 采集 → 显示
result_queue = queue.Queue(maxsize=2)   # 算法 → 串口

running = True

# 显示线程用的最新结果
latest_center   = None     # 靶心坐标
latest_laser_pt = None     # 激光点预测坐标
latest_distance = -1.0     # 估算距离
state_lock = threading.Lock()


def estimate_distance(area_px):
    if area_px <= 0:
        return -1.0
    return REFERENCE_DISTANCE_MM * np.sqrt(REFERENCE_AREA_PIXELS / area_px)


def calc_laser_point(distance_mm):
    d = max(MIN_DISTANCE_MM, min(MAX_DISTANCE_MM, distance_mm))
    y = LASER_Y_MIN_DISTANCE + (d - MIN_DISTANCE_MM) * \
        (LASER_Y_MAX_DISTANCE - LASER_Y_MIN_DISTANCE) / \
        (MAX_DISTANCE_MM - MIN_DISTANCE_MM)
    return (int(LASER_X_FIXED), int(round(y)))


# ============================================================
# 线程 1：采集
# ============================================================
def thread_capture():
    print("[DEBUG] 采集线程启动")
    cap = Camera()
    if not cap.open():
        print("Camera open failed")
        return

    count, start, fps = 0, time.time(), 0.0

    while running:
        frame = cap.capture()
        if frame is None:
            continue

        count += 1
        now = time.time()
        if now - start >= 1.0:
            fps = count / (now - start)
            print(f"[采集 FPS]: {fps:.2f}")
            count, start = 0, now

        if frame_queue.full():
            try: frame_queue.get_nowait()
            except: pass
        frame_queue.put_nowait(frame)

        if frame_queue2.full():
            try: frame_queue2.get_nowait()
            except: pass
        frame_queue2.put_nowait((frame, fps))

    cap.close()


# ============================================================
# 线程 2：算法（靶心 + 距离 + 激光点）
# ============================================================
def thread_algo():
    global latest_center, latest_laser_pt, latest_distance

    # 面积平滑窗口，降低距离抖动
    area_win = []
    WIN = 5

    while running:
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        result = CenterGet(frame, return_area=True)

        if result is None:
            with state_lock:
                latest_center = None
                latest_laser_pt = None
                latest_distance = -1.0
            if result_queue.full():
                try: result_queue.get_nowait()
                except: pass
            result_queue.put_nowait((None, None, -1.0))
            continue

        center, area = result

        # 面积平滑
        area_win.append(area)
        if len(area_win) > WIN:
            area_win.pop(0)
        smooth_area = float(np.mean(area_win))

        distance = estimate_distance(smooth_area)
        laser_pt = calc_laser_point(distance) if distance > 0 else None

        with state_lock:
            latest_center   = center
            latest_laser_pt = laser_pt
            latest_distance = distance

        if result_queue.full():
            try: result_queue.get_nowait()
            except: pass
        result_queue.put_nowait((center, laser_pt, distance))


# ============================================================
# 线程 3：串口（激光点 - 靶心 → dx/dy）
# ============================================================
def thread_serial():
    ser = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not ser.open():
        print("Serial open failed")
        return

    last_send = time.time()

    while running:
        try:
            center, laser_pt, distance = result_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        now = time.time()
        if now - last_send < SEND_INTERVAL:
            continue

        if center is not None and laser_pt is not None:
            # 误差 = 激光点 - 靶心
            # 目的：让云台带动激光点移动，最终激光点与靶心重合
            # y 方向保持和原版一致（图像 y 下为正，云台 y 上为正，所以取反）
            dx = float(laser_pt[0] - center[0])
            dy = float(-(laser_pt[1] - center[1]))
            ser.send_deta(dx, dy)
            print(f"发送: dx={dx:+6.2f}, dy={dy:+6.2f}  "
                  f"laser={laser_pt}  target={center}  D={distance:.0f}mm")
        else:
            ser.send_deta(0.0, 0.0)

        last_send = now

    ser.close()


# ============================================================
# 主线程：显示
# ============================================================
def main():
    global running

    t1 = threading.Thread(target=thread_capture, daemon=True)
    t2 = threading.Thread(target=thread_algo,    daemon=True)
    t3 = threading.Thread(target=thread_serial,  daemon=True)
    t1.start(); t2.start(); t3.start()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"点击坐标: ({x}, {y})")

    cv2.namedWindow("laser_track")
    cv2.setMouseCallback("laser_track", on_mouse)

    disp_cnt, disp_start = 0, time.time()
    disp_fps = 0.0

    try:
        while True:
            try:
                frame, cap_fps = frame_queue2.get_nowait()
            except queue.Empty:
                cv2.waitKey(1)
                continue

            disp_cnt += 1
            now = time.time()
            if now - disp_start >= 1.0:
                disp_fps = disp_cnt / (now - disp_start)
                disp_cnt, disp_start = 0, now

            with state_lock:
                center   = latest_center
                laser_pt = latest_laser_pt
                distance = latest_distance

            # 画面几何中心（参考）
            cv2.line(frame, (305, 240), (335, 240), (80, 80, 80), 1)
            cv2.line(frame, (320, 225), (320, 255), (80, 80, 80), 1)

            # 靶心（红实心）
            if center is not None:
                cv2.circle(frame, center, 6, (0, 0, 255), -1)

            # 激光点预测（黄十字 + 圆圈）
            if laser_pt is not None:
                cv2.drawMarker(frame, laser_pt, (0, 255, 255),
                               cv2.MARKER_CROSS, 22, 2)
                cv2.circle(frame, laser_pt, 10, (0, 255, 255), 2)

            # 误差连线
            if center is not None and laser_pt is not None:
                err_color = (0, 255, 0) if abs(laser_pt[0]-center[0]) < 5 and \
                                           abs(laser_pt[1]-center[1]) < 5 \
                                        else (0, 255, 255)
                cv2.line(frame, laser_pt, center, err_color, 1)

            # 文字信息
            cv2.putText(frame, f"FPS cap:{cap_fps:.1f} disp:{disp_fps:.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            if distance > 0:
                cv2.putText(frame, f"Dist: {distance:.0f} mm",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            if center is not None and laser_pt is not None:
                dx = +(laser_pt[0] - center[0])
                dy = -(laser_pt[1] - center[1])
                cv2.putText(frame, f"dx={dx:+d} dy={dy:+d}",
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            cv2.imshow("laser_track", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    running = False
    time.sleep(0.5)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
