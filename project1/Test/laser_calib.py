# -*- coding: utf-8 -*-
"""
激光补偿标定脚本

用途：
  在不同距离下实时显示检测面积、估算距离、激光点Y坐标，
  帮助你确定以下三个参数的实际值：
    REFERENCE_AREA_PIXELS  —— 靶板在 1500mm 处的像素面积
    LASER_Y_MIN_DISTANCE   —— 500mm 处激光实际落点 Y 坐标
    LASER_Y_MAX_DISTANCE   —— 1500mm 处激光实际落点 Y 坐标

操作说明：
  鼠标左键点击画面中激光实际落点 → 记录该点 Y 坐标
  按 'r' 清除点击点
  按 's' 保存当前帧到 /tmp/calib_<timestamp>.jpg
  按 'q' 退出

标定流程：
  1. 将靶板放在 1500mm 处，等待面积稳定，记录 "面积" 数值 → REFERENCE_AREA_PIXELS
  2. 将小车移到 500mm 处，打开激光，点击激光落点 → 记录 Y → LASER_Y_MIN_DISTANCE
  3. 将小车移到 1500mm 处，点击激光落点 → 记录 Y → LASER_Y_MAX_DISTANCE
"""

import os
import sys
import time
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from Drivers.camera import Camera
from Algorithm.CenterGet import CenterGet

# ============================================================
# 待标定参数（先填入初始猜测值，标定后更新）
# ============================================================
REFERENCE_AREA_PIXELS = 8412    # 靶板外框在 1500mm 处的像素面积（待标定）
REFERENCE_DISTANCE_MM = 1500    # 标定距离，固定 1500mm

MIN_DISTANCE_MM = 500
MAX_DISTANCE_MM = 1500

LASER_X_FIXED        = 322      # 激光点 X 坐标（由安装位置决定，一般不变）
LASER_Y_MIN_DISTANCE = 240      # 500mm 处激光落点 Y（待标定）
LASER_Y_MAX_DISTANCE = 250      # 1500mm 处激光落点 Y（待标定）

# 面积平滑窗口大小
AREA_SMOOTH_N = 10

# ============================================================
# 参数文件路径（标定结果自动保存到这里）
# ============================================================
PARAM_FILE = os.path.join(os.path.dirname(__file__), "laser_calib_params.json")


def estimate_distance(area_pixels):
    if area_pixels <= 0:
        return -1.0
    return REFERENCE_DISTANCE_MM * np.sqrt(REFERENCE_AREA_PIXELS / area_pixels)


def calc_laser_y(distance_mm):
    d = max(MIN_DISTANCE_MM, min(MAX_DISTANCE_MM, distance_mm))
    return LASER_Y_MIN_DISTANCE + (d - MIN_DISTANCE_MM) * \
        (LASER_Y_MAX_DISTANCE - LASER_Y_MIN_DISTANCE) / (MAX_DISTANCE_MM - MIN_DISTANCE_MM)


def save_params(params: dict):
    with open(PARAM_FILE, "w", encoding="utf-8") as f:
        json.dump(params, f, indent=2, ensure_ascii=False)
    print(f"[标定] 参数已保存到 {PARAM_FILE}")


def load_params():
    if os.path.exists(PARAM_FILE):
        with open(PARAM_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def draw_overlay(frame, area, distance, laser_y, clicked_pt, area_history):
    h, w = frame.shape[:2]

    # 画面中心参考线（BASE_POINT）
    cx, cy = w // 2, h // 2
    cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

    # 激光点预测位置（基于当前参数）
    if distance > 0:
        pred_x = LASER_X_FIXED
        pred_y = int(calc_laser_y(distance))
        cv2.drawMarker(frame, (pred_x, pred_y), (0, 255, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, "pred", (pred_x + 8, pred_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # 鼠标点击的激光实际落点
    if clicked_pt is not None:
        cv2.drawMarker(frame, clicked_pt, (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, f"actual Y={clicked_pt[1]}",
                    (clicked_pt[0] + 8, clicked_pt[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # 面积历史曲线（右侧小图）
    if len(area_history) > 1:
        bar_w, bar_h = 120, 60
        bar_x, bar_y = w - bar_w - 10, 10
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                      (40, 40, 40), -1)
        max_a = max(area_history) if max(area_history) > 0 else 1
        pts = []
        for i, a in enumerate(area_history):
            px = bar_x + int(i * bar_w / len(area_history))
            py = bar_y + bar_h - int(a / max_a * bar_h)
            pts.append((px, py))
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i - 1], pts[i], (0, 200, 255), 1)

    # 文字信息面板
    lines = [
        f"Area  : {area:.0f} px2" if area > 0 else "Area  : --",
        f"Dist  : {distance:.1f} mm" if distance > 0 else "Dist  : --",
        f"LaserY: {calc_laser_y(distance):.1f} px" if distance > 0 else "LaserY: --",
        "",
        f"[Params]",
        f"REF_AREA={REFERENCE_AREA_PIXELS}",
        f"Y_min={LASER_Y_MIN_DISTANCE}  Y_max={LASER_Y_MAX_DISTANCE}",
        f"X_fixed={LASER_X_FIXED}",
        "",
        "LClick=mark laser  r=reset  s=save  q=quit",
    ]
    for i, line in enumerate(lines):
        color = (200, 200, 200) if not line.startswith("[") else (100, 255, 100)
        cv2.putText(frame, line, (10, 25 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)


def main():
    global REFERENCE_AREA_PIXELS, LASER_Y_MIN_DISTANCE, LASER_Y_MAX_DISTANCE, LASER_X_FIXED

    # 尝试加载上次保存的参数
    saved = load_params()
    if saved:
        REFERENCE_AREA_PIXELS = saved.get("REFERENCE_AREA_PIXELS", REFERENCE_AREA_PIXELS)
        LASER_Y_MIN_DISTANCE  = saved.get("LASER_Y_MIN_DISTANCE",  LASER_Y_MIN_DISTANCE)
        LASER_Y_MAX_DISTANCE  = saved.get("LASER_Y_MAX_DISTANCE",  LASER_Y_MAX_DISTANCE)
        LASER_X_FIXED         = saved.get("LASER_X_FIXED",         LASER_X_FIXED)
        print(f"[标定] 已加载上次参数: {saved}")

    cap = Camera()
    if not cap.open():
        print("Camera open failed")
        return

    clicked_pt = None
    area_history = []

    def on_mouse(event, x, y, flags, param):
        nonlocal clicked_pt
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_pt = (x, y)
            print(f"[标定] 点击坐标: ({x}, {y})  → 激光落点 Y = {y}")

    cv2.namedWindow("laser_calib")
    cv2.setMouseCallback("laser_calib", on_mouse)

    print("=" * 50)
    print("激光补偿标定脚本启动")
    print("标定流程：")
    print("  1. 靶板放 1500mm → 记录 Area 值 → 填入 REFERENCE_AREA_PIXELS")
    print("  2. 小车移 500mm  → 点击激光落点 → 记录 Y → LASER_Y_MIN_DISTANCE")
    print("  3. 小车移 1500mm → 点击激光落点 → 记录 Y → LASER_Y_MAX_DISTANCE")
    print("=" * 50)

    try:
        while True:
            frame = cap.capture()
            if frame is None:
                continue

            result = CenterGet(frame, return_area=True)

            area = 0.0
            if result is not None:
                center, area = result
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            area_history.append(area)
            if len(area_history) > AREA_SMOOTH_N:
                area_history.pop(0)

            smooth_area = np.mean([a for a in area_history if a > 0]) if any(a > 0 for a in area_history) else 0.0
            distance = estimate_distance(smooth_area)

            draw_overlay(frame, smooth_area, distance, calc_laser_y(distance) if distance > 0 else 0,
                         clicked_pt, area_history)

            cv2.imshow("laser_calib", frame)

            key = cv2.waitKey(1) & 0xFF
            if key != 255 and key != 0xFF:
                # 任意按键都回显一下，方便判断焦点是否在窗口上
                print(f"[按键] key={key} ({chr(key) if 32 <= key < 127 else '?'})")
            if key == ord('q'):
                break
            elif key == ord('r'):
                clicked_pt = None
                print("[标定] 已清除点击点")
            elif key == ord('s'):
                ts = int(time.time())
                path = f"/tmp/calib_{ts}.jpg"
                cv2.imwrite(path, frame)
                print(f"[标定] 已保存帧: {path}")
            elif key == ord('1'):
                # 将当前平滑面积设为参考面积
                if smooth_area > 0:
                    REFERENCE_AREA_PIXELS = int(smooth_area)
                    print(f"[标定] 设置 REFERENCE_AREA_PIXELS = {REFERENCE_AREA_PIXELS}")
                    save_params({
                        "REFERENCE_AREA_PIXELS": REFERENCE_AREA_PIXELS,
                        "LASER_Y_MIN_DISTANCE":  LASER_Y_MIN_DISTANCE,
                        "LASER_Y_MAX_DISTANCE":  LASER_Y_MAX_DISTANCE,
                        "LASER_X_FIXED":         LASER_X_FIXED,
                    })
                else:
                    print("[标定] 没有检测到靶板（面积=0），无法记录参考面积")
            elif key == ord('2'):
                # 将点击点 Y 设为 LASER_Y_MIN_DISTANCE（500mm 处）
                if clicked_pt is not None:
                    LASER_Y_MIN_DISTANCE = clicked_pt[1]
                    print(f"[标定] 设置 LASER_Y_MIN_DISTANCE = {LASER_Y_MIN_DISTANCE}")
                    save_params({
                        "REFERENCE_AREA_PIXELS": REFERENCE_AREA_PIXELS,
                        "LASER_Y_MIN_DISTANCE":  LASER_Y_MIN_DISTANCE,
                        "LASER_Y_MAX_DISTANCE":  LASER_Y_MAX_DISTANCE,
                        "LASER_X_FIXED":         LASER_X_FIXED,
                    })
                else:
                    print("[标定] 请先鼠标左键点击激光落点")
            elif key == ord('3'):
                # 将点击点 Y 设为 LASER_Y_MAX_DISTANCE（1500mm 处）
                if clicked_pt is not None:
                    LASER_Y_MAX_DISTANCE = clicked_pt[1]
                    print(f"[标定] 设置 LASER_Y_MAX_DISTANCE = {LASER_Y_MAX_DISTANCE}")
                    save_params({
                        "REFERENCE_AREA_PIXELS": REFERENCE_AREA_PIXELS,
                        "LASER_Y_MIN_DISTANCE":  LASER_Y_MIN_DISTANCE,
                        "LASER_Y_MAX_DISTANCE":  LASER_Y_MAX_DISTANCE,
                        "LASER_X_FIXED":         LASER_X_FIXED,
                    })
                else:
                    print("[标定] 请先鼠标左键点击激光落点")
            elif key == ord('4'):
                # 将点击点 X 设为 LASER_X_FIXED
                if clicked_pt is not None:
                    LASER_X_FIXED = clicked_pt[0]
                    print(f"[标定] 设置 LASER_X_FIXED = {LASER_X_FIXED}")
                    save_params({
                        "REFERENCE_AREA_PIXELS": REFERENCE_AREA_PIXELS,
                        "LASER_Y_MIN_DISTANCE":  LASER_Y_MIN_DISTANCE,
                        "LASER_Y_MAX_DISTANCE":  LASER_Y_MAX_DISTANCE,
                        "LASER_X_FIXED":         LASER_X_FIXED,
                    })
                else:
                    print("[标定] 请先鼠标左键点击激光落点")

    except KeyboardInterrupt:
        pass
    finally:
        cap.close()
        cv2.destroyAllWindows()
        print("\n[标定] 最终参数：")
        print(f"  REFERENCE_AREA_PIXELS = {REFERENCE_AREA_PIXELS}")
        print(f"  LASER_Y_MIN_DISTANCE  = {LASER_Y_MIN_DISTANCE}")
        print(f"  LASER_Y_MAX_DISTANCE  = {LASER_Y_MAX_DISTANCE}")
        print(f"  LASER_X_FIXED         = {LASER_X_FIXED}")


if __name__ == "__main__":
    main()
