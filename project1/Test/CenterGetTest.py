import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
import time
import math

from Drivers.camera import Camera
from Algorithm.CenterGet import CenterGet


def test():
    cap = Camera()
    cap.open()
    st = time.time()

    # 帧率计算变量
    # fps = 0
    # frame_count = 0
    # fps_start_time = time.time()

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
            # frame_count += 1
            # current_time = time.time()
            # elapsed_time = current_time - fps_start_time
            # if elapsed_time >= 1.0:  # 每1秒更新一次帧率
            #     fps = frame_count / elapsed_time
            #     frame_count = 0
            #     fps_start_time = current_time

            center = CenterGet(frame)
            print(f'center:{center}')
            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # 在图像上显示帧率
            # cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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