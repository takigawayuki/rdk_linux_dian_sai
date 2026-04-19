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
            center = CenterGet(frame)
            print(f'center:{center}')
            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
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