import cv2
import numpy as np
import matplotlib.pyplot as plt

class CircleGet:
    def __init__(self):
        # A4纸物理尺寸(mm)
        self.a4_width = 265  # 宽210mm
        self.a4_height = 203  # 高297mm
        self.circle_radius = 60  # 圆半径6cm = 60mm
        
        # 透视变换矩阵
        self.M = None
        # 圆的16个等分点（物理坐标，单位mm）
        self.physical_circle_points = self._generate_physical_circle_points()
    
    def _generate_physical_circle_points(self, divisions=64):
        """生成A4纸物理坐标系下圆的16个等分点"""
        points = []
        # 圆心在A4纸中心
        center_x = self.a4_width / 2
        center_y = self.a4_height / 2
        
        for i in range(divisions):
            angle = 2 * np.pi * i / divisions
            # 计算物理坐标(mm)
            x = center_x + self.circle_radius * np.cos(angle)
            y = center_y + self.circle_radius * np.sin(angle)
            points.append([x, y])
        
        return np.float32(points)
    
    def calculate_perspective_matrix(self, detected_corners):
        """
        计算透视变换矩阵
        detected_corners: 检测到的A4纸四个角点(图像坐标)
                          顺序要求：左上、右上、右下、左下
        """
        # A4纸物理坐标(mm)，顺序与检测点对应
        physical_corners = np.float32([
            [0, 0],              # 左上
            [self.a4_width, 0],  # 右上
            [self.a4_width, self.a4_height],  # 右下
            [0, self.a4_height]  # 左下
        ])
        
        # 检测到的图像坐标
        image_corners = np.float32(detected_corners)
        
        # 计算透视变换矩阵: 物理坐标 → 图像坐标
        self.M = cv2.getPerspectiveTransform(physical_corners, image_corners)
        return self.M
    
    def transform_circle_points(self):
        """将物理坐标系下的16个点变换到图像坐标系"""
        if self.M is None:
            raise ValueError("请先调用calculate_perspective_matrix计算变换矩阵")
        
        # 转换为齐次坐标
        points_homogeneous = np.hstack([
            self.physical_circle_points,
            np.ones((len(self.physical_circle_points), 1), dtype=np.float32)
        ])
        
        # 应用透视变换
        transformed_points = np.dot(self.M, points_homogeneous.T).T
        
        # 转换回二维坐标
        transformed_points = transformed_points[:, :2] / transformed_points[:, 2:]
        
        return transformed_points.astype(np.int32)
    
    def pts_ordered(self, pts):
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


    def circle_points_offset(self, center, raw_circle_points):
        center_circle = np.mean(raw_circle_points, axis=0)
        center_circle = center_circle.astype(int)
        # 计算偏移量
        offset = center - center_circle
        # 对circle_points进行偏移
        circle_points = raw_circle_points + offset

        return circle_points


    def forward(self, center, pts):
        pts = pts.reshape(4, 2).astype(int)
        pts = self.pts_ordered(pts)
        self.calculate_perspective_matrix(pts)
        circle_points = self.transform_circle_points()
        center_circle = np.mean(circle_points, axis=0)
        center_circle = center_circle.astype(int)
        # 计算偏移量
        offset = center - center_circle
        # 对circle_points进行偏移
        circle_points = circle_points + offset

        return circle_points


    def visualize(self, image, detected_corners, transformed_points):
        """可视化结果"""
        vis_img = image.copy()
        
        # 绘制检测到的A4纸边框
        cv2.polylines(vis_img, [np.array(detected_corners, dtype=np.int32)],
                      isClosed=True, color=(0, 255, 0), thickness=2)
        
        # 绘制变换后的16个点
        for i, (x, y) in enumerate(transformed_points):
            # 第一个点用红色标记，其余用蓝色
            color = (0, 0, 255) if i == 0 else (255, 0, 0)
            cv2.circle(vis_img, (x, y), 5, color, -1)
            cv2.putText(vis_img, f"{i}", (x+8, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 显示结果
        plt.figure(figsize=(12, 8))
        plt.imshow(cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB))
        plt.title("A4纸上圆的16个点透视变换结果")
        plt.axis('off')
        plt.show()

# 使用示例
if __name__ == "__main__":
    # 示例：检测到的A4纸四个角点（图像坐标，需按左上→右上→右下→左下顺序）
    # 实际应用中从你的检测算法获取
    detected_corners = [
        [180, 150],   # 左上
        [520, 130],   # 右上
        [550, 480],   # 右下
        [150, 500]    # 左下
    ]
    
    # 读取图像（替换为你的图像路径）
    try:
        image = cv2.imread("a4_image.jpg")
        if image is None:
            raise FileNotFoundError
    except:
        # 若图像不存在，创建空白图像用于演示
        image = np.ones((600, 700, 3), dtype=np.uint8) * 255
    
    # 创建转换器实例
    transformer = CircleGet()
    
    # 计算透视变换矩阵
    transformer.calculate_perspective_matrix(detected_corners)
    
    # 计算变换后的16个点
    circle_points = transformer.transform_circle_points()
    
    # 打印结果
    print("图像中圆上16个点的坐标：")
    for i, (x, y) in enumerate(circle_points):
        print(f"点{i:2d}: ({x:4d}, {y:4d})")
    
    # 可视化
    transformer.visualize(image, detected_corners, circle_points)
