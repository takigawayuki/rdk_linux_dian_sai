import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
import time
import math

from Drivers.camera import Camera
from Algorithm.CenterGet import CenterGet


def print_camera_info():
    """打印摄像头系统参数，用于排查帧率问题"""
    print("=" * 50)
    print("[摄像头系统诊断]")

    # 用 v4l2-ctl 查询摄像头支持的格式和帧率
    ret = os.system("which v4l2-ctl > /dev/null 2>&1")
    if ret == 0:
        print("\n-- 摄像头设备列表 --")
        os.system("v4l2-ctl --list-devices 2>/dev/null")
        print("\n-- 支持的格式与帧率 (/dev/video0) --")
        os.system("v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null")
    else:
        print("[提示] 未安装 v4l2-ctl，跳过系统级查询（可用 sudo apt install v4l-utils 安装）")

    # 用 Camera 类打开后读取实际生效的参数（与运行时一致）
    print("\n-- OpenCV 读取到的实际参数（经 Camera 类设置后）--")
    _cam = Camera()
    if _cam.open():
        cap = _cam.cvcap
        w   = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h   = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        fmt = cap.get(cv2.CAP_PROP_FOURCC)
        fourcc_str = "".join([chr(int(fmt) >> (8 * i) & 0xFF) for i in range(4)])
        print(f"  分辨率  : {int(w)} x {int(h)}")
        print(f"  帧率    : {fps} fps  ← 这是驱动上报值，不一定是实际值")
        print(f"  像素格式: {fourcc_str}")
        _cam.close()
    else:
        print("  无法打开摄像头")

    print("=" * 50)


def test():
    print_camera_info()
    cap = Camera()
    cap.open()
    st = time.time()

    # 帧率计算变量
    fps = 0
    frame_count = 0
    fps_start_time = time.time()

    # 设置视频编码格式
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('output.avi', fourcc, 90.0, (640, 360))

    try:
        while True:
            print(f'cost:{time.time() - st}s')
            st = time.time()
            frame = cap.capture()
            if frame is None:
                print("无法获取图像帧")
                break

            # 计算帧率
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - fps_start_time
            if elapsed_time >= 1.0:  # 每1秒更新一次帧率
                fps = frame_count / elapsed_time
                frame_count = 0
                fps_start_time = current_time

            center = CenterGet(frame)
            print(f'center:{center}')
            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # 在图像上显示帧率
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # out.write(frame)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    cap.close()
    # out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test()