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

BASE_POINT = (320, 180)

SERIAL_PORT = "/dev/ttyS1"
BAUDRATE = 115200
SEND_INTERVAL = 0.05

# ===== 队列（关键修改）=====
frame_queue   = queue.Queue(maxsize=5)   # 采集 → 算法
result_queue  = queue.Queue(maxsize=5)   # 算法 → 串口
display_queue = queue.Queue(maxsize=5)   # 算法 → 显示

running = True


# =========================
# 线程1：摄像头采集（高FPS）
# =========================
def thread_capture():
    cap = Camera()
    if not cap.open():
        print("Camera open failed")
        return

    while running:
        frame = cap.capture()
        if frame is None:
            continue

        # 👉 丢旧帧（关键！）
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except:
                pass

        frame_queue.put_nowait(frame)

    cap.close()


# =========================
# 线程2：算法（瓶颈）
# =========================
def thread_algo():
    while running:
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        center = CenterGet(frame)

        # 👉 给串口（只传center）
        if result_queue.full():
            try:
                result_queue.get_nowait()
            except:
                pass
        result_queue.put_nowait(center)

        # 👉 给显示（frame + center）
        if display_queue.full():
            try:
                display_queue.get_nowait()
            except:
                pass
        display_queue.put_nowait((frame, center))


# =========================
# 线程3：串口发送（低频）
# =========================
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


# =========================
# 主线程：显示 + FPS统计
# =========================
def main():
    global running

    t1 = threading.Thread(target=thread_capture, daemon=True)
    t2 = threading.Thread(target=thread_algo, daemon=True)
    t3 = threading.Thread(target=thread_serial, daemon=True)

    t1.start()
    t2.start()
    t3.start()

    fps = 0
    count = 0
    start = time.time()

    try:
        while True:
            try:
                frame, center = display_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # FPS统计（真实显示帧率）
            count += 1
            now = time.time()
            if now - start >= 1.0:
                fps = count / (now - start)
                count = 0
                start = now
                print(f"FPS: {fps:.2f}")

            # 画图
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