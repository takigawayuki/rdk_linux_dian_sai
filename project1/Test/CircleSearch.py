import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
import time
import math

from Drivers.camera import Camera
from Algorithm.CenterGet import CenterGet
from Algorithm.CircleGet import CircleGet


def order_points(pts):
    """
    对四个角点进行排序，顺序为：右上、左上、左下、右下
    
    参数:
    pts -- 一个4x2的数组，包含四个点的坐标
    
    返回:
    排序后的点列表
    """
    # 初始化排序后的坐标列表
    rect = np.zeros((4, 2), dtype="float32")
    
    s = pts.sum(axis=1)
    rect[3] = pts[np.argmax(s)]  # 右下角 - x+y最大
    rect[1] = pts[np.argmin(s)]  # 左上角 - x+y最小
    
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(diff)]  # 右上角 - y-x最小
    rect[2] = pts[np.argmax(diff)]  # 左下角 - y-x最大

    if rect[0][1] > rect[1][1] and rect[2][1] > rect[3][1]:
        swap = rect[0][1]
        rect[0][1] = rect[1][1]
        rect[1][1] = swap
        swap = rect[2][1]
        rect[2][1] = rect[3][1]
        rect[3][1] = swap
    
    # 返回排序后的坐标
    return rect



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
            results = CenterGet(frame, return_pts=True)

            if results is not None:
                center = results[0]
                pts = results[1].reshape(4, 2).astype(int)
                pts = order_points(pts)

                circle = CircleGet()
                circle.calculate_perspective_matrix(pts)
                circle_points = circle.transform_circle_points()
                # 需要对circle_points进行根据center偏移
                # 需要计算circle_points平均中心到center的距离
                center_circle = np.mean(circle_points, axis=0)
                center_circle = center_circle.astype(int)
                # 计算偏移量
                offset = center - center_circle
                # 对circle_points进行偏移
                circle_points = circle_points + offset

                for point in pts:
                    cv2.circle(frame, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
                for point in circle_points[1:]:
                    cv2.circle(frame, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)
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