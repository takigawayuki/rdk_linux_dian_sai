import cv2 as cv


class Camera:
    def __init__(self):
        # 初始化摄像头对象
        self.cvcap = None
        self.is_opened = False

    # def open(self, main_size=(640, 360)):
    def open(self, main_size=(640, 480), fps=120):
        # 如果摄像头已经打开，则直接返回True
        if self.is_opened: return True

        try:
            # 创建OpenCV摄像头对象，使用默认摄像头(0)
            # self.cvcap = cv.VideoCapture(0)

            self.cvcap = cv.VideoCapture(0, cv.CAP_V4L2)

            self.cvcap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
            
            # 设置分辨率
            self.cvcap.set(cv.CAP_PROP_FRAME_WIDTH, main_size[0])
            self.cvcap.set(cv.CAP_PROP_FRAME_HEIGHT, main_size[1])

            self.cvcap.set(cv.CAP_PROP_FPS, fps)

            # 检查摄像头是否成功打开
            if not self.cvcap.isOpened():
                raise Exception("Failed to open camera")
            
            # ✅ 打印真实参数（非常关键）
            print("====== Camera Info ======")
            print("Resolution:",
                self.cvcap.get(cv.CAP_PROP_FRAME_WIDTH),
                self.cvcap.get(cv.CAP_PROP_FRAME_HEIGHT))
            print("FPS (setting):",
                self.cvcap.get(cv.CAP_PROP_FPS))
            print("FOURCC:",
                int(self.cvcap.get(cv.CAP_PROP_FOURCC)))
            print("=========================")

            # 设置摄像头已打开标志
            self.is_opened = True
            return True

        except Exception as e:
            # 打印摄像头打开失败的错误信息
            print(f"Camera Open Failed: {str(e)}")

            # 设置摄像头未打开标志
            self.is_opened = False
            return False
        
    def capture(self, resize=None):
        # 如果摄像头未打开，则返回None
        if not self.is_opened: return None

        try:
            # 捕获图像
            ret, frame = self.cvcap.read()
            # 检查是否成功捕获图像
            if not ret or frame is None:
                return None
            # 添加图像旋转180度的代码
            frame = cv.rotate(frame, cv.ROTATE_180)
            # 调整大小（如BlackSearch中使用的640x480）
            if resize and isinstance(resize, tuple) and len(resize) == 2:
                frame = cv.resize(frame, resize)
            # 返回捕获到的图像
            return frame
        # 捕获异常并打印错误信息
        except Exception as e:
            print(f"Image Capture Failed: {str(e)}")
            # 返回None
            return None

    def close(self):
        if self.is_opened and self.cvcap is not None:
            self.cvcap.release()
            self.is_opened = False

    def __del__(self):
        # 确保对象销毁时释放资源
        self.close()
