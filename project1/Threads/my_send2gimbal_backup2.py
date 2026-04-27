import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import time
import threading
import queue

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet

# BASE_POINT = (320, 180)  # 摄像头分辨率是 640x360
BASE_POINT = (320, 240)    # 摄像头分辨率是 640x480

SERIAL_PORT = "/dev/ttyS1"
BAUDRATE = 115200
SEND_INTERVAL = 0.05

frame_queue  = queue.Queue(maxsize=2)   # 采集 → 算法
frame_queue2 = queue.Queue(maxsize=2)   # 采集 → 显示（独立通道，不等算法）
result_queue = queue.Queue(maxsize=2)   # 算法 → 串口

running = True
latest_center = None          # 算法结果，显示线程异步读取
center_lock   = threading.Lock()


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

        if frame_queue.full():
            try: frame_queue.get_nowait()
            except: pass
        frame_queue.put_nowait(frame)

        if frame_queue2.full():
            try: frame_queue2.get_nowait()
            except: pass
        frame_queue2.put_nowait((frame, fps))  # 传 (frame, fps) 元组

    cap.close()


def thread_algo():
    global latest_center
    while running:
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        center = CenterGet(frame)

        with center_lock:
            latest_center = center

        if result_queue.full():
            try: result_queue.get_nowait()
            except: pass
        result_queue.put_nowait(center)


def thread_serial():
    serial = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not serial.open():
        print("Serial open failed")
        return

    last_send_time = time.time()

    while running:
        try:
            center = result_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        now = time.time()
        if now - last_send_time < SEND_INTERVAL:
            continue

        if center is not None:
            dx = -(BASE_POINT[0] - center[0]) * 0.001
            dy = +(BASE_POINT[1] - center[1]) * 0.001
            serial.send_deta(dx, dy)
            print(f"发送: dx={dx:.4f}, dy={dy:.4f}")
        else:
            serial.send_data("DETA:0.0000,0.0000\n")

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
                frame, fps = frame_queue2.get(timeout=0.1)
            except queue.Empty:
                continue

            display_count += 1
            now = time.time()
            if now - display_start >= 1.0:
                display_fps = display_count / (now - display_start)
                print(f"[显示 FPS]: {display_fps:.2f}")
                display_count = 0
                display_start = now

            with center_lock:
                center = latest_center

            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            cv2.circle(frame, BASE_POINT, 3, (0, 255, 0), -1)
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