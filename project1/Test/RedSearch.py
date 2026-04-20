import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
from Drivers.camera import Camera

# 创建窗口
cv2.namedWindow('RedSearch')
cv2.namedWindow('Binary Mask')

# 初始化HSV滑动条（单范围检测，所有参数均可调）
cv2.createTrackbar('Hue Min', 'RedSearch', 0, 179, lambda x: None)
cv2.createTrackbar('Hue Max', 'RedSearch', 179, 179, lambda x: None)
cv2.createTrackbar('Saturation Min', 'RedSearch', 0, 255, lambda x: None)
cv2.createTrackbar('Saturation Max', 'RedSearch', 255, 255, lambda x: None)
cv2.createTrackbar('Value Min', 'RedSearch', 0, 255, lambda x: None)
cv2.createTrackbar('Value Max', 'RedSearch', 255, 255, lambda x: None)

# 初始化摄像头
camera = Camera()
camera.open()

if not camera.is_opened:
    print("无法打开摄像头")
    exit()

try:
    while True:
        # 获取图像
        frame = camera.capture()
        if frame is None:
            break

        # 调整图像大小
        frame = cv2.resize(frame, (640, 360))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 获取滑动条值
        h_min = cv2.getTrackbarPos('Hue Min', 'RedSearch')
        h_max = cv2.getTrackbarPos('Hue Max', 'RedSearch')
        s_min = cv2.getTrackbarPos('Saturation Min', 'RedSearch')
        s_max = cv2.getTrackbarPos('Saturation Max', 'RedSearch')
        v_min = cv2.getTrackbarPos('Value Min', 'RedSearch')
        v_max = cv2.getTrackbarPos('Value Max', 'RedSearch')

        # 创建单一范围的HSV掩码
        lower_red = np.array([h_min, s_min, v_min])
        upper_red = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # 形态学操作优化掩码
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 获取红色区域的所有像素坐标
        red_points = np.where(mask > 0)

        # 每隔10个像素绘制一个淡红色点
        if len(red_points[0]) > 0:
            step = 10
            y_coords = red_points[0][::step]
            x_coords = red_points[1][::step]
            for x, y in zip(x_coords, y_coords):
                cv2.circle(frame, (x, y), 2, (255, 180, 180), -1)

        # 显示图像
        cv2.imshow('RedSearch', frame)
        # 显示二值化掩码（红色为白，其余为黑）
        cv2.imshow('Binary Mask', mask)

        # 退出条件
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.close()
    cv2.destroyAllWindows()