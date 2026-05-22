# -*- coding: utf-8 -*-
"""
CenterKalman —— 靶心 (x, y) 卡尔曼滤波，专为 CenterGet 偶尔丢识别设计。

状态向量: [px, py, vx, vy]^T   恒速模型
测量向量: [px, py]^T

用法：
    kf = CenterKalman(dt=1/110, R=4.0, Q=20.0)

    每帧识别成功：
        smooth = kf.update((cx, cy))      # 返回滤波后 (x, y)
    每帧识别失败（且初始化过、跟丢未超时）：
        smooth = kf.predict_only()         # 纯预测，不修正
    跟丢太久：
        kf.reset()
"""
import numpy as np
import cv2 as cv


class CenterKalman:
    def __init__(self, dt=1.0 / 110.0, R=4.0, Q=20.0):
        self.dt = dt
        self.R_val = R
        self.Q_val = Q
        self.kf = cv.KalmanFilter(4, 2)
        self._build_matrices()
        self.initialized = False

    def _build_matrices(self):
        dt = self.dt
        # 状态转移矩阵 F：恒速模型
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        # 测量矩阵 H：只测位置
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        # 测量噪声 R：相信测量到什么程度，越小越信
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * self.R_val
        # 过程噪声 Q：允许靶子动得多快，越大越敢跟
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * self.Q_val
        # 初始误差协方差 P：稍大表示初始不确定
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 1.0

    def update_dt(self, dt):
        """如需用真实帧间隔可调用此方法"""
        self.dt = dt
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

    def reset(self):
        self.initialized = False
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 1.0
        self.kf.statePost = np.zeros((4, 1), dtype=np.float32)

    def update(self, point):
        """识别到测量值，返回滤波后的 (x, y)"""
        z = np.array([[float(point[0])], [float(point[1])]], dtype=np.float32)
        if not self.initialized:
            # 首次：状态 = 测量位置，速度 0
            self.kf.statePost = np.array(
                [[z[0, 0]], [z[1, 0]], [0.0], [0.0]],
                dtype=np.float32,
            )
            self.initialized = True
            return int(round(z[0, 0])), int(round(z[1, 0]))
        self.kf.predict()
        est = self.kf.correct(z)
        return int(round(est[0, 0])), int(round(est[1, 0]))

    def predict_only(self):
        """识别失败时调用，返回纯预测 (x, y)。未初始化返回 None。"""
        if not self.initialized:
            return None
        est = self.kf.predict()
        # 把预测当成新的后验，下一帧 predict 才是基于这个继续推
        self.kf.statePost = est.copy()
        return int(round(est[0, 0])), int(round(est[1, 0]))
