import cv2
import time
import argparse


def camera_test():
    """
    test camera with video
    """
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    number = 0
    # 读取帧
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 旋转图像180度
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        cv2.imshow("USB Camera", frame)
        print("Frame number:%d, time: %d", number, time.time())
        number += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()


def video_record_by_frame():
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 设置视频编码格式
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        if not ret:
            break
        # 旋转图像180度
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # 显示图像
        cv2.imshow('frame', frame)

        # 写入视频文件
        out.write(frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()

def picture_record_by_click():
    """
    once click, get and save a frame
    frame name is timestamp
    """
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Press 's' to save a picture, 'q' to quit.")

    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        if not ret:
            break
        # 旋转图像180度
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # 显示图像
        cv2.imshow('frame', frame)

        # 按 's' 键保存图片，按 'q' 键退出
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            timestamp = time.strftime("%m%d_%H%M%S")
            cv2.imwrite(f'{timestamp}.jpg', frame)
            print(f"Saved {timestamp}.jpg")
        elif key == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

def main():
    """
    main function
    """
    parser = argparse.ArgumentParser(description="camera test")
    parser.add_argument("--task", type=str, default="video_record_by_frame", help="task name")
    args = parser.parse_args()
    task_name = args.task

    if task_name == "video_record_by_frame":
        video_record_by_frame()
    elif task_name == "picture_record_by_click":
        picture_record_by_click()
    elif task_name == "camera_test":
        camera_test()
    else:
        print("task name error")


if __name__ == "__main__":
    main()