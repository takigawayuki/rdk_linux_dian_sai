import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import time
import threading
import queue

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet
from Algorithm.KalmanFilter2D import KalmanFilter2D

BASE_POINT = (320, 240)
SERIAL_PORT = "/dev/ttyS1"
BAUDRATE = 115200
SEND_INTERVAL = 0.01  # 100Hz
MAX_LOST_FRAMES = 10

frame_queue  = queue.Queue(maxsize=2)   # 采集 → 算法
frame_queue2 = queue.Queue(maxsize=2)   # 采集 → 显示
result_queue = queue.Queue(maxsize=2)   # 算法 → 串口

running = True
latest_raw = None
latest_filtered = None
center_lock = threading.Lock()


def thread_capture():
    cap = Camera()
    if not cap.open():
        print("Camera open failed")
        return

    count = 0
    start = time.time()
    fps = 0.0

    while running:
        frame = cap.capture()
        if frame is None:
            continue

        count += 1
        now = time.time()
        if now - start >= 1.0:
            fps = count / (now - start)
            print(f"[采集 FPS]: {fps:.2f}")
            count = 0
            start = now

        # 喂给算法线程
        if frame_queue.full():
            try: frame_queue.get_nowait()
            except: pass
        frame_queue.put_nowait(frame)

        # 喂给显示线程
        if frame_queue2.full():
            try: frame_queue2.get_nowait()
            except: pass
        frame_queue2.put_nowait((frame, fps))

    cap.close()


def thread_algo():
    global latest_raw, latest_filtered
    kalman = KalmanFilter2D(npoints=1, dt=SEND_INTERVAL)
    last_measurement = None
    lost_frames = 0

    while running:
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        raw_center = CenterGet(frame)

        with center_lock:
            latest_raw = raw_center

        if raw_center is not None:
            measurement = np.array([[raw_center[0]], [raw_center[1]]], dtype=np.float32)
            filtered = kalman.predict(measurement)
            last_measurement = measurement
            lost_frames = 0
            fc = (int(filtered[0][0]), int(filtered[0][1]))
        else:
            lost_frames += 1
            if lost_frames > MAX_LOST_FRAMES or last_measurement is None:
                last_measurement = None
                fc = None
            else:
                filtered = kalman.predict(last_measurement)
                fc = (int(filtered[0][0]), int(filtered[0][1]))

        with center_lock:
            latest_filtered = fc

        if result_queue.full():
            try: result_queue.get_nowait()
            except: pass
        result_queue.put_nowait(fc)


def thread_serial():
    serial = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not serial.open():
        print("Serial open failed")
        return

    last_send_time = time.time()

    while running:
        try:
            fc = result_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        now = time.time()
        if now - last_send_time < SEND_INTERVAL:
            continue

        if fc is not None:
            deta_x = -(BASE_POINT[0] - fc[0])
            deta_y = +(BASE_POINT[1] - fc[1])
            serial.send_deta(deta_x, deta_y)
            print(f"发送: deta_x={deta_x:.2f}, deta_y={deta_y:.2f}")
        else:
            serial.send_deta(0.0, 0.0)

        last_send_time = now

    serial.close()


def main():
    global running

    t1 = threading.Thread(target=thread_capture, daemon=True)
    t2 = threading.Thread(target=thread_algo,    daemon=True)
    t3 = threading.Thread(target=thread_serial,  daemon=True)

    t1.start()
    t2.start()
    t3.start()

    display_count = 0
    display_start = time.time()

    try:
        while True:
            try:
                frame, fps = frame_queue2.get_nowait()
            except queue.Empty:
                cv2.waitKey(1)
                continue

            display_count += 1
            now = time.time()
            if now - display_start >= 1.0:
                display_fps = display_count / (now - display_start)
                print(f"[显示 FPS]: {display_fps:.2f}")
                display_count = 0
                display_start = now

            with center_lock:
                raw_center = latest_raw
                fc = latest_filtered

            if raw_center is not None:
                cv2.circle(frame, raw_center, 5, (0, 0, 255), -1)  # 红：原始检测
            if fc is not None:
                cv2.circle(frame, fc, 5, (0, 255, 0), -1)          # 绿：卡尔曼滤波
            cv2.circle(frame, BASE_POINT, 3, (255, 0, 0), -1)      # 蓝：基准点
            cv2.putText(frame, f"FPS:{fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    running = False
    time.sleep(0.5)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
