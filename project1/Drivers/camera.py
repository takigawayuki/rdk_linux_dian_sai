import cv2 as cv
import os


class Camera:
    def __init__(self):
        self.cvcap = None
        self.is_opened = False

    def open(self, main_size=(640, 360)):
        if self.is_opened:
            return True

        try:
            # 👉 默认用 /dev/video0
            self.cvcap = cv.VideoCapture(0)

            if not self.cvcap.isOpened():
                print("Failed to open camera")
                return False

            # 🔥 关键：切 MJPG（提升性能）
            self.cvcap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))

            # 分辨率（保持你原来的接口）
            self.cvcap.set(cv.CAP_PROP_FRAME_WIDTH, main_size[0])
            self.cvcap.set(cv.CAP_PROP_FRAME_HEIGHT, main_size[1])

            # ❌ 不要强行设 FPS（驱动会忽略）
            # self.cvcap.set(cv.CAP_PROP_FPS, 90)

            self.is_opened = True
            print("Using USB camera (/dev/video0)")
            return True

        except Exception as e:
            print(f"Camera Open Failed: {str(e)}")
            self.is_opened = False
            return False

    def capture(self, resize=None):
        if not self.is_opened:
            return None

        try:
            ret, frame = self.cvcap.read()
            if not ret:
                return None

            # 保留你原来的逻辑
            frame = cv.rotate(frame, cv.ROTATE_180)

            if resize and isinstance(resize, tuple) and len(resize) == 2:
                frame = cv.resize(frame, resize)

            return frame

        except Exception as e:
            print(f"Image Capture Failed: {str(e)}")
            return None

    def close(self):
        if self.is_opened:
            if self.cvcap:
                self.cvcap.release()
            self.is_opened = False

    def __del__(self):
        self.close()