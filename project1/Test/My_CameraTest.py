import cv2
import time

# 打开摄像头
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# ====== 强制设置（关键） ======
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 120)

# ====== 打印实际参数（确认是否生效） ======
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps_set = cap.get(cv2.CAP_PROP_FPS)

fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])

print("====== Camera Info ======")
print(f"Resolution: {int(width)}x{int(height)}")
print(f"FPS (setting): {fps_set}")
print(f"FOURCC: {fourcc_str}")
print("=========================")

# ====== FPS统计 ======
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("读取失败")
        break

    frame_count += 1

    # 每秒打印一次真实FPS
    if time.time() - start_time >= 1.0:
        print(f"Real FPS: {frame_count}")
        frame_count = 0
        start_time = time.time()

    # ⚠️ 显示（会限制帧率，但方便观察）
    cv2.imshow("Camera Test (Press q to quit)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()