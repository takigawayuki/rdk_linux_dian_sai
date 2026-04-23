import cv2
import numpy as np
import math


# 二值化阈值调整
threshold_value = 83
# 添加Canny边缘检测的阈值参数
canny_low_threshold = 50
canny_high_threshold = 150


def preprocess_image(img):
    """
    :param img: 输入图像
    :return: 预处理后的图像
    :description: 对图像进行预处理，包括灰度化、高斯模糊、二值化、Canny边缘检测
    """
    # 转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # 二值化
    _, thresh = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY)
    return thresh

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

def CenterGet(img, return_pts=False):
    """
    :param img: 输入图像
    :return: 中心坐标
    :description: 获取图像中心坐标,暂时没有考虑透视
    """
    frame = preprocess_image(img)
    # Canny边缘检测
    edges = cv2.Canny(frame, canny_low_threshold, canny_high_threshold)
    # 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

    if best_contour is not None:
        if best_center is not None:
            if return_pts:
                return best_center, best_approx
            return best_center
    else:
        return None
    