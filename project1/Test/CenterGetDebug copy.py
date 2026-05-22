import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import math
import json

from Drivers.camera import Camera

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'center_get_params.json')

WIN_ORIG   = 'Original'
WIN_THRESH = 'Threshold'
WIN_EDGES  = 'Edges'
WIN_RESULT = 'Result (contours)'
WIN_CTRL   = 'Controls'

def nothing(_): pass

def load_params():
    """从配置文件加载参数，如果文件不存在则返回默认值"""
    defaults = {
        'threshold': 144,
        'canny_low': 50,
        'canny_high': 150,
        'min_area': 500,
        'blur_kernel': 5
    }
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                params = json.load(f)
                print(f'已加载参数: {CONFIG_FILE}')
                return params
        except:
            print(f'加载参数失败，使用默认值')
            return defaults
    return defaults

def save_params(thresh, canny_low, canny_high, min_area, blur_k):
    """保存当前参数到配置文件"""
    params = {
        'threshold': thresh,
        'canny_low': canny_low,
        'canny_high': canny_high,
        'min_area': min_area,
        'blur_kernel': blur_k
    }
    with open(CONFIG_FILE, 'w') as f:
        json.dump(params, f, indent=2)
    print(f'参数已保存到 {CONFIG_FILE}')

def update_centerget_file(thresh, canny_low, canny_high, min_area):
    """直接更新 CenterGet.py 的全局变量"""
    centerget_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'Algorithm', 'CenterGet.py'
    )
    try:
        with open(centerget_path, 'r') as f:
            lines = f.readlines()

        with open(centerget_path, 'w') as f:
            for line in lines:
                if line.startswith('threshold_value ='):
                    f.write(f'threshold_value = {thresh}\n')
                elif line.startswith('canny_low_threshold ='):
                    f.write(f'canny_low_threshold = {canny_low}\n')
                elif line.startswith('canny_high_threshold ='):
                    f.write(f'canny_high_threshold = {canny_high}\n')
                elif 'if area <' in line:
                    indent = line[:len(line) - len(line.lstrip())]
                    f.write(f'{indent}if area < {min_area}:\n')
                else:
                    f.write(line)
        print(f'已更新 CenterGet.py 的参数')
    except Exception as e:
        print(f'更新 CenterGet.py 失败: {e}')

def setup_trackbars():
    params = load_params()
    cv2.namedWindow(WIN_CTRL, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_CTRL, 500, 250)
    cv2.createTrackbar('Threshold',   WIN_CTRL,  params['threshold'], 255, nothing)
    cv2.createTrackbar('Canny Low',   WIN_CTRL,   params['canny_low'], 500, nothing)
    cv2.createTrackbar('Canny High',  WIN_CTRL,  params['canny_high'], 500, nothing)
    cv2.createTrackbar('Min Area',    WIN_CTRL,  params['min_area'], 5000, nothing)
    cv2.createTrackbar('Blur Kernel', WIN_CTRL,    params['blur_kernel'],  21, nothing)

def get_trackbar_values():
    thresh     = cv2.getTrackbarPos('Threshold',   WIN_CTRL)
    canny_low  = cv2.getTrackbarPos('Canny Low',   WIN_CTRL)
    canny_high = cv2.getTrackbarPos('Canny High',  WIN_CTRL)
    min_area   = cv2.getTrackbarPos('Min Area',    WIN_CTRL)
    blur_k     = cv2.getTrackbarPos('Blur Kernel', WIN_CTRL)
    # 保证 blur kernel 为正奇数
    blur_k = max(1, blur_k)
    if blur_k % 2 == 0:
        blur_k += 1
    return thresh, canny_low, canny_high, min_area, blur_k

def draw_values(img, thresh, canny_low, canny_high, min_area, blur_k):
    lines = [
        f'Threshold : {thresh}',
        f'Canny Low : {canny_low}',
        f'Canny High: {canny_high}',
        f'Min Area  : {min_area}',
        f'Blur Kern : {blur_k}',
    ]
    y = 20
    for line in lines:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 1, cv2.LINE_AA)
        y += 22

def calculate_equidistant_center(pts):
    pts = np.array(pts, dtype=np.float32)
    if len(pts) != 4:
        return None
    diag1_start, diag1_end = pts[0], pts[2]
    diag2_start, diag2_end = pts[1], pts[3]
    a1 = diag1_end[1] - diag1_start[1]
    b1 = diag1_start[0] - diag1_end[0]
    c1 = diag1_end[0] * diag1_start[1] - diag1_start[0] * diag1_end[1]
    a2 = diag2_end[1] - diag2_start[1]
    b2 = diag2_start[0] - diag2_end[0]
    c2 = diag2_end[0] * diag2_start[1] - diag2_start[0] * diag2_end[1]
    denom = a1 * b2 - a2 * b1
    if denom != 0:
        x = (b1 * c2 - b2 * c1) / denom
        y = (a2 * c1 - a1 * c2) / denom
    else:
        x = np.mean(pts[:, 0])
        y = np.mean(pts[:, 1])
    return (int(round(x)), int(round(y)))

def process(frame, thresh_val, canny_low, canny_high, min_area, blur_k):
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)
    _, thresh_img = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY)
    edges   = cv2.Canny(thresh_img, canny_low, canny_high)

    result  = frame.copy()
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score   = -1
    best_center  = None
    best_approx  = None
    h, w = thresh_img.shape[:2]

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        # 画出所有通过面积过滤的轮廓（灰色）
        cv2.drawContours(result, [contour], -1, (80, 80, 80), 1)

        perimeter = cv2.arcLength(contour, True)
        epsilon   = 0.01 * perimeter
        approx    = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) != 4:
            continue

        pts = approx.reshape(4, 2).astype(int)
        border_threshold = 5
        if not all(border_threshold < pt[0] < w - border_threshold and
                   border_threshold < pt[1] < h - border_threshold for pt in pts):
            continue

        angles = []
        for i in range(4):
            p_prev = pts[(i - 1) % 4]
            p_curr = pts[i]
            p_next = pts[(i + 1) % 4]
            vec1   = p_prev - p_curr
            vec2   = p_next - p_curr
            angle  = math.degrees(math.atan2(vec2[1], vec2[0]) - math.atan2(vec1[1], vec1[0]))
            angle  = abs(angle)
            if angle > 180:
                angle = 360 - angle
            angles.append(angle)

        if not all(70 < angle < 110 for angle in angles):
            # 画出角度不合格的四边形（蓝色）
            cv2.drawContours(result, [approx], -1, (255, 100, 0), 1)
            continue

        lengths = []
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            lengths.append(math.sqrt((x2 - x1)**2 + (y2 - y1)**2))

        if max(lengths) / min(lengths) > 5:
            continue

        angle_deviation = sum(abs(a - 90) for a in angles) / 4
        angle_score     = 100 - angle_deviation
        max_possible    = (w * h) / 2
        area_score      = min(100, (area / max_possible) * 100)
        total_score     = 0.6 * angle_score + 0.4 * area_score

        # 画出候选四边形（黄色）
        cv2.drawContours(result, [approx], -1, (0, 255, 255), 1)

        if total_score > best_score:
            best_score   = total_score
            best_contour = contour
            best_approx  = approx
            M = cv2.moments(contour)
            if M['m00'] != 0:
                best_center = calculate_equidistant_center(pts)

    if best_contour is not None:
        # 最佳轮廓绿色高亮
        cv2.drawContours(result, [best_approx], -1, (0, 255, 0), 2)
        if best_center:
            cv2.circle(result, best_center, 6, (0, 0, 255), -1)
            cv2.putText(result, f'{best_center}', (best_center[0] + 8, best_center[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    status = f'Center: {best_center}  Score: {best_score:.1f}' if best_center else 'Center: None'
    cv2.putText(result, status, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 把阈值数字叠加到二值图和边缘图上（方便截图对比）
    thresh_bgr = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)
    edges_bgr  = cv2.cvtColor(edges,      cv2.COLOR_GRAY2BGR)
    draw_values(thresh_bgr, thresh_val, canny_low, canny_high, min_area, blur_k)
    draw_values(edges_bgr,  thresh_val, canny_low, canny_high, min_area, blur_k)

    return thresh_bgr, edges_bgr, result


def main():
    cap = Camera()
    cap.open()

    setup_trackbars()
    cv2.namedWindow(WIN_ORIG,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_THRESH, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_EDGES,  cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_RESULT, cv2.WINDOW_NORMAL)

    print('按 Q 或 Ctrl+C 退出，自动保存参数并写入 CenterGet.py')
    print('颜色说明: 灰=通过面积  蓝=角度不合格  黄=候选  绿=最佳')

    last_params = (0, 0, 0, 0, 0)
    try:
        while True:
            frame = cap.capture()
            if frame is None:
                print('无法获取图像帧')
                break

            t, cl, ch, ma, bk = get_trackbar_values()
            last_params = (t, cl, ch, ma, bk)
            thresh_img, edges_img, result_img = process(frame, t, cl, ch, ma, bk)

            cv2.imshow(WIN_ORIG,   frame)
            cv2.imshow(WIN_THRESH, thresh_img)
            cv2.imshow(WIN_EDGES,  edges_img)
            cv2.imshow(WIN_RESULT, result_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        t, cl, ch, ma, bk = last_params
        save_params(t, cl, ch, ma, bk)
        update_centerget_file(t, cl, ch, ma)

    cap.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
