import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import numpy as np
import time

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet

# 基准点（激光点坐标）
# BASE_POINT = (291, 201)     # Laser point coordinates, 640*360 resolution only
BASE_POINT = (320, 180)       # 640/2, 360/2

# 串口配置
SERIAL_PORT = "/dev/ttyS1"  # Windows端口，Linux下改为"/dev/ttyAMA0"
BAUDRATE = 115200

# 发送频率控制
'''
### 频率与间隔的对应关系
- 10Hz ： SEND_INTERVAL = 0.1 （100ms）
- 20Hz ： SEND_INTERVAL = 0.05 （50ms，当前值）
- 30Hz ： SEND_INTERVAL = 0.033 （约33ms）
- 50Hz ： SEND_INTERVAL = 0.02 （20ms）
'''
SEND_INTERVAL = 0.05  # 50ms发送一次

def main():
    # 初始化摄像头
    cap = Camera()
    if not cap.open():
        print("Failed to open camera")
        return

    # 初始化串口
    serial = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not serial.open():
        print("Failed to open serial port")
        cap.close()
        return

    last_send_time = time.time()
    
    # 帧率计算变量
    fps = 0
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # 获取图像
            frame = cap.capture()
            if frame is None:
                print("Failed to get image frame")
                time.sleep(0.01)
                continue

            # 计算帧率
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 1.0:  # 每1秒计算一次帧率
                fps = frame_count / elapsed_time
                frame_count = 0
                start_time = current_time

            # 目标检测
            center = CenterGet(frame)

            current_time = time.time()
            # 控制发送频率
            if current_time - last_send_time >= SEND_INTERVAL:
                if center is not None:
                    # 计算偏差
                    deta_x = -(BASE_POINT[0] - center[0]) * 0.001
                    deta_y = +(BASE_POINT[1] - center[1]) * 0.001
                    
                    # 发送偏差值给云台
                    success = serial.send_deta(deta_x, deta_y)
                    if success:
                        print(f"发送偏差值: deta_x={deta_x:.4f}, deta_y={deta_y:.4f}")
                    else:
                        print("发送失败")
                else:
                    # 未检测到目标，发送特殊信号
                    serial.send_data("DETA:0.0000,0.0000\n")
                    print("未检测到目标")
                
                last_send_time = current_time

            # 显示图像（可选）
            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.circle(frame, BASE_POINT, 3, (0, 255, 0), -1)
            
            # 显示帧率
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Program interrupted manually")
    finally:
        # 清理资源
        cap.close()
        serial.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()