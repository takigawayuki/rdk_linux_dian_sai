import cv2

cap = cv2.VideoCapture(0)  # 对应 /dev/video0

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("读取失败")
        break

    cv2.imshow("Camera Test", frame)

    # 按 q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()