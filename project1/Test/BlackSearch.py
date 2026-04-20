import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
import math

from Drivers.camera import Camera


# 全局变量用于阈值调整
threshold_value = 197

# 添加Canny边缘检测的阈值参数
canny_low_threshold = 50
canny_high_threshold = 150

def on_threshold_change(val):
    """阈值调整回调函数"""
    global threshold_value
    # 将传入的值赋给全局变量threshold_value
    threshold_value = val

# 计算到四个角点距离相等的中心点
def calculate_equidistant_center(pts):
    # 确保pts是4x2的数组
    pts = np.array(pts, dtype=np.float32)
    if len(pts) != 4:
        return None

    # 对于矩形，理想情况下对角线交点到四个角点距离相等
    # 计算两条对角线
    diag1_start, diag1_end = pts[0], pts[2]
    diag2_start, diag2_end = pts[1], pts[3]

    # 计算两条对角线的交点
    # 直线方程: ax + by + c = 0
    a1 = diag1_end[1] - diag1_start[1]
    b1 = diag1_start[0] - diag1_end[0]
    c1 = diag1_end[0] * diag1_start[1] - diag1_start[0] * diag1_end[1]

    a2 = diag2_end[1] - diag2_start[1]
    b2 = diag2_start[0] - diag2_end[0]
    c2 = diag2_end[0] * diag2_start[1] - diag2_start[0] * diag2_end[1]

    # 计算交点
    denom = a1 * b2 - a2 * b1
    if denom != 0:
        x = (b1 * c2 - b2 * c1) / denom
        y = (a2 * c1 - a1 * c2) / denom
    else:
        # 如果对角线平行，则计算几何中心
        x = np.mean(pts[:, 0])
        y = np.mean(pts[:, 1])

    # 四舍五入到最近的像素
    return (int(round(x)), int(round(y)))

def detect_black_frame():
    # 初始化摄像头（参考CameraTest.py）
    picam2 = Camera()
    # 打开摄像头
    picam2.open()

    # 创建窗口和阈值滑条
    cv2.namedWindow('BlackSearch')
    cv2.createTrackbar('gray threshold', 'BlackSearch', threshold_value, 255, on_threshold_change)

    while True:
        # 读取摄像头帧（CameraTest风格）
        frame = picam2.capture()
        if frame is None:
            print("无法获取图像帧")
            break

        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 应用阈值处理
        _, thresh = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)

        # 应用Canny边缘检测
        edges = cv2.Canny(thresh, canny_low_threshold, canny_high_threshold)

        # 创建一个用于显示轮廓的图像
        contour_frame = np.zeros_like(frame)

        # 寻找轮廓 (从Canny边缘图像)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制所有轮廓 (蓝色)
        cv2.drawContours(contour_frame, contours, -1, (255, 0, 0), 2)

        # 初始化最佳轮廓变量
        best_contour = None
        best_score = -1
        best_center = None
        best_approx = None

        # 处理每个轮廓以寻找类矩形
        for contour in contours:
            # 过滤面积过小的轮廓
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            # 轮廓近似
            perimeter = cv2.arcLength(contour, True)
            epsilon = 0.01 * perimeter
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 检查是否为四边形
            if len(approx) == 4:
                # 提取角点坐标
                pts = approx.reshape(4, 2).astype(int)

                # 边界检查
                border_threshold = 5
                h, w = frame.shape[:2]
                if not all(border_threshold < pt[0] < w - border_threshold and
                           border_threshold < pt[1] < h - border_threshold for pt in pts):
                    continue

                # 计算角度
                angles = []
                for i in range(4):
                    p_prev = pts[(i - 1) % 4]
                    p_curr = pts[i]
                    p_next = pts[(i + 1) % 4]

                    vec1 = p_prev - p_curr
                    vec2 = p_next - p_curr

                    angle = math.degrees(math.atan2(vec2[1], vec2[0]) - math.atan2(vec1[1], vec1[0]))
                    angle = abs(angle)
                    if angle > 180:
                        angle = 360 - angle
                    angles.append(angle)

                # 检查角度是否在合理范围内
                if all(70 < angle < 110 for angle in angles):
                    # 计算边长比例
                    lengths = []
                    for i in range(4):
                        x1, y1 = pts[i]
                        x2, y2 = pts[(i + 1) % 4]
                        length = math.sqrt((x2 - x1)**2 + (y2 - y1)** 2)
                        lengths.append(length)

                    max_len = max(lengths)
                    min_len = min(lengths)

                    if max_len / min_len <= 5:
                        # 计算角度得分 (与90度的平均偏差，越小越好)
                        angle_deviation = sum(abs(angle - 90) for angle in angles) / 4
                        angle_score = 100 - angle_deviation  # 满分100分

                        # 计算面积得分 (归一化处理，越大越好)
                        max_possible_area = (w * h) / 2  # 假设最大可能面积为图像面积的一半
                        area_score = min(100, (area / max_possible_area) * 100)

                        # 综合得分 (角度权重60%，面积权重40%)
                        total_score = 0.6 * angle_score + 0.4 * area_score

                        # 更新最佳轮廓
                        if total_score > best_score:
                            best_score = total_score
                            best_contour = contour
                            best_approx = approx

                            # 计算中心点
                            M = cv2.moments(contour)
                            if M['m00'] != 0:
                                best_center = calculate_equidistant_center(pts)
                            else:
                                best_center = None

        # 显示最佳轮廓
        if best_contour is not None:
            # 绘制最佳轮廓 (绿色)
            cv2.drawContours(frame, [best_contour], -1, (0, 255, 0), 3)

            # 绘制角点
            pts = best_approx.reshape(4, 2).astype(int)
            for i, pt in enumerate(pts):
                cv2.circle(frame, tuple(pt), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"P{i+1}", (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 显示中心点
            if best_center is not None:
                cv2.circle(frame, best_center, 5, (255, 0, 0), -1)
                cv2.putText(frame, f"中心: {best_center}", (best_center[0] + 10, best_center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # 显示得分
            cv2.putText(frame, f"得分: {best_score:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 绘制四角点
        if best_approx is not None:
            for i, corner in enumerate(best_approx):
                x, y = corner[0]
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"{i+1}:({x},{y})", (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "未找到目标", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 显示结果
        # 显示Canny边缘检测结果
        cv2.imshow('Canny Edges', edges)
        # 显示轮廓检测结果
        cv2.imshow('Contours', contour_frame)
        # 显示原始帧
        cv2.imshow('BlackSearch', frame)
        # 显示灰度阈值处理结果
        cv2.imshow('gray threshold', thresh)

        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    picam2.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_black_frame()
