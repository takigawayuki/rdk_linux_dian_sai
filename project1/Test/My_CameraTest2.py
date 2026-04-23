import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 120)

frame_count = 0
start = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    if time.time() - start >= 1:
        print("Real FPS:", frame_count)
        frame_count = 0
        start = time.time()