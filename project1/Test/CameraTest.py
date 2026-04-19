import cv2
import time


def camera_test():
    cap = cv2.VideoCapture(0)

    # 🔥 切 MJPG（关键）
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # 分辨率（你可以改）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    number = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        frame = cv2.rotate(frame, cv2.ROTATE_180)

        cv2.imshow("USB Camera", frame)

        print(f"Frame number: {number}, time: {time.time()}")
        number += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_test()