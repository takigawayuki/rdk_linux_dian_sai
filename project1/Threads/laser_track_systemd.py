# -*- coding: utf-8 -*-
"""
激光点映射导航（激光补偿版）—— systemd 后台自启动版

与 laser_track.py 的区别：
  - 移除所有 OpenCV 窗口 / 显示线程 / 鼠标回调 / waitKey
  - 主线程仅负责启动子线程并等待 SIGTERM/SIGINT，收到后让线程优雅退出
  - 用 logging 输出日志（systemd journal 友好），并对高频发送做 1Hz 限频
  - 适合 systemd 服务、上电后台运行（无 DISPLAY、无 X11）

思路：
  1. CenterGet 检测靶板外框，拿中心坐标 + 轮廓面积
  2. 面积 → 距离：D = D_ref × sqrt(A_ref / A)
  3. 距离 → 激光点坐标：X 固定，Y 在 [Y_min, Y_max] 线性插值
  4. 串口发送 (dx, dy) = 激光点 - 靶心
"""
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import time
import signal
import logging
import threading
import queue
import numpy as np

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet

# ============================================================
# 日志（systemd 会把 stdout/stderr 收进 journal）
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
    stream=sys.stdout,
)
log = logging.getLogger("laser_track")

# ============================================================
# 串口 / 帧率
# ============================================================
SERIAL_PORT   = "/dev/ttyS1"
BAUDRATE      = 115200
SEND_INTERVAL = 0.01        # 100Hz 发送
LOG_INTERVAL  = 1.0         # 1Hz 打日志，避免 journal 被刷爆

# ============================================================
# 标定参数（从 laser_calib_params.json 自动加载，没有就用默认）
# ============================================================
REFERENCE_AREA_PIXELS = 7756
REFERENCE_DISTANCE_MM = 1320
MIN_DISTANCE_MM       = 710
MAX_DISTANCE_MM       = 1320
LASER_X_FIXED         = 318
LASER_Y_MIN_DISTANCE  = 222
LASER_Y_MAX_DISTANCE  = 230

PARAM_FILE = os.path.join(os.path.dirname(__file__), "laser_calib_params.json")
if os.path.exists(PARAM_FILE):
    try:
        with open(PARAM_FILE, "r", encoding="utf-8") as f:
            _p = json.load(f)
        REFERENCE_AREA_PIXELS = _p.get("REFERENCE_AREA_PIXELS", REFERENCE_AREA_PIXELS)
        LASER_X_FIXED         = _p.get("LASER_X_FIXED",         LASER_X_FIXED)
        LASER_Y_MIN_DISTANCE  = _p.get("LASER_Y_MIN_DISTANCE",  LASER_Y_MIN_DISTANCE)
        LASER_Y_MAX_DISTANCE  = _p.get("LASER_Y_MAX_DISTANCE",  LASER_Y_MAX_DISTANCE)
        log.info("参数已从 %s 加载", os.path.basename(PARAM_FILE))
    except Exception as e:
        log.warning("加载参数失败，使用默认值：%s", e)

log.info("REF: A=%dpx @ D=%dmm", REFERENCE_AREA_PIXELS, REFERENCE_DISTANCE_MM)
log.info("距离范围: %d~%dmm", MIN_DISTANCE_MM, MAX_DISTANCE_MM)
log.info("激光: X=%d, Y=%d~%d", LASER_X_FIXED, LASER_Y_MIN_DISTANCE, LASER_Y_MAX_DISTANCE)

# ============================================================
# 队列 / 共享状态
# ============================================================
frame_queue  = queue.Queue(maxsize=2)   # 采集 → 算法
result_queue = queue.Queue(maxsize=2)   # 算法 → 串口

running = True


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
    log.info("采集线程启动")
    cap = Camera()
    if not cap.open():
        log.error("Camera open failed")
        return

    count, start, fps = 0, time.time(), 0.0

    try:
        while running:
            frame = cap.capture()
            if frame is None:
                continue

            count += 1
            now = time.time()
            if now - start >= 5.0:    # 5 秒打一次采集 FPS
                fps = count / (now - start)
                log.info("采集 FPS: %.2f", fps)
                count, start = 0, now

            if frame_queue.full():
                try: frame_queue.get_nowait()
                except queue.Empty: pass
            try:
                frame_queue.put_nowait(frame)
            except queue.Full:
                pass
    finally:
        cap.close()
        log.info("采集线程退出")


# ============================================================
# 线程 2：算法（靶心 + 距离 + 激光点）
# ============================================================
def thread_algo():
    log.info("算法线程启动")

    area_win = []
    WIN = 5

    algo_count, algo_start = 0, time.time()

    while running:
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        result = CenterGet(frame, return_area=True)

        algo_count += 1
        now = time.time()
        if now - algo_start >= 5.0:
            algo_fps = algo_count / (now - algo_start)
            log.info("算法 FPS: %.2f", algo_fps)
            algo_count, algo_start = 0, now

        if result is None:
            if result_queue.full():
                try: result_queue.get_nowait()
                except queue.Empty: pass
            try:
                result_queue.put_nowait((None, None, -1.0))
            except queue.Full:
                pass
            continue

        center, area = result

        area_win.append(area)
        if len(area_win) > WIN:
            area_win.pop(0)
        smooth_area = float(np.mean(area_win))

        distance = estimate_distance(smooth_area)
        laser_pt = calc_laser_point(distance) if distance > 0 else None

        if result_queue.full():
            try: result_queue.get_nowait()
            except queue.Empty: pass
        try:
            result_queue.put_nowait((center, laser_pt, distance))
        except queue.Full:
            pass

    log.info("算法线程退出")


# ============================================================
# 线程 3：串口（激光点 - 靶心 → dx/dy）
# ============================================================
def thread_serial():
    log.info("串口线程启动")
    ser = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not ser.open():
        log.error("Serial open failed")
        return

    last_send = time.time()
    last_log  = 0.0

    try:
        while running:
            try:
                center, laser_pt, distance = result_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            now = time.time()
            if now - last_send < SEND_INTERVAL:
                continue

            if center is not None and laser_pt is not None:
                dx = float(laser_pt[0] - center[0])
                dy = float(-(laser_pt[1] - center[1]))
                ser.send_deta(dx, dy)
                if now - last_log >= LOG_INTERVAL:
                    log.info("发送 dx=%+6.2f dy=%+6.2f laser=%s target=%s D=%.0fmm",
                             dx, dy, laser_pt, center, distance)
                    last_log = now
            else:
                ser.send_deta(0.0, 0.0)
                if now - last_log >= LOG_INTERVAL:
                    log.info("未识别到目标，发送 0,0")
                    last_log = now

            last_send = now
    finally:
        ser.close()
        log.info("串口线程退出")


# ============================================================
# 信号处理：systemctl stop / Ctrl+C 都走这里
# ============================================================
def _on_signal(signum, _frame):
    global running
    log.info("收到信号 %d，准备退出", signum)
    running = False


def main():
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT,  _on_signal)

    threads = [
        threading.Thread(target=thread_capture, name="capture", daemon=True),
        threading.Thread(target=thread_algo,    name="algo",    daemon=True),
        threading.Thread(target=thread_serial,  name="serial",  daemon=True),
    ]
    for t in threads:
        t.start()

    log.info("主线程进入等待，pid=%d", os.getpid())
    while running:
        time.sleep(0.5)

    # 给子线程一点时间收尾
    deadline = time.time() + 2.0
    for t in threads:
        remaining = max(0.0, deadline - time.time())
        t.join(timeout=remaining)

    log.info("主线程退出")


if __name__ == "__main__":
    main()
