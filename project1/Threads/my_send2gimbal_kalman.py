import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import time
import threading
import queue
import multiprocessing as mp

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet
from Algorithm.KalmanFilter2D import KalmanFilter2D

BASE_POINT = (320, 240)
SERIAL_PORT = "/dev/ttyS1"
BAUDRATE = 115200
SEND_INTERVAL = 0.01  # 100Hz
MAX_LOST_FRAMES = 10


def algo_process(capture_q, result_q):
    """算法进程：独立进程跑 CenterGet + Kalman，绕开 GIL"""
    kalman = KalmanFilter2D(npoints=1, dt=SEND_INTERVAL)
    last_measurement = None
    lost_frames = 0

    while True:
        item = capture_q.get()
        if item is None:  # 退出信号
            break

        frame, fps = item
        raw_center = CenterGet(frame)

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

        try:
            result_q.put_nowait((frame, fps, raw_center, fc))
        except:
            pass  # 队列满就丢弃


def main():
    # 进程间队列（采集 → 算法，算法 → 主线程）
    capture_q = mp.Queue(maxsize=1)
    result_q = mp.Queue(maxsize=2)

    # 线程间队列（主线程 → 串口线程）
    serial_q = queue.Queue(maxsize=2)

    # 启动算法进程
    proc = mp.Process(target=algo_process, args=(capture_q, result_q), daemon=True)
    proc.start()

    # 初始化摄像头
    cap = Camera()
    if not cap.open():
        print("Camera open failed")
        return

    # 初始化串口
    serial = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not serial.open():
        print("Serial open failed")
        cap.close()
        return

    running = True

    # 采集线程
    def thread_capture():
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

            try:
                capture_q.put_nowait((frame, fps))
            except:
                pass  # 队列满就丢弃旧帧

    # 串口线程
    def thread_serial():
        last_send_time = time.time()

        while running:
            try:
                fc = serial_q.get(timeout=0.1)
            except queue.Empty:
                continue

            now = time.time()
            if now - last_send_time < SEND_INTERVAL:
                continue

            if fc is not None:
                deta_x = -(BASE_POINT[0] - fc[0]) * 0.001
                deta_y = +(BASE_POINT[1] - fc[1]) * 0.001
                serial.send_deta(deta_x, deta_y)
                print(f"发送: deta_x={deta_x:.4f}, deta_y={deta_y:.4f}")
            else:
                serial.send_data("DETA:0.0000,0.0000\n")

            last_send_time = now

    t1 = threading.Thread(target=thread_capture, daemon=True)
    t2 = threading.Thread(target=thread_serial, daemon=True)
    t1.start()
    t2.start()

    display_count = 0
    display_start = time.time()

    try:
        while True:
            try:
                frame, fps, raw_center, fc = result_q.get_nowait()
            except:
                cv2.waitKey(1)
                continue

            # 转发给串口线程
            try:
                serial_q.put_nowait(fc)
            except:
                pass

            display_count += 1
            now = time.time()
            if now - display_start >= 1.0:
                display_fps = display_count / (now - display_start)
                print(f"[显示 FPS]: {display_fps:.2f}")
                display_count = 0
                display_start = now

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
    capture_q.put(None)  # 通知算法进程退出
    proc.join(timeout=2)
    cap.close()
    serial.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)  # Linux 下用 spawn 避免 fork 问题
    main()
