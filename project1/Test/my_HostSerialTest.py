import os 
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import threading

from Drivers.my_serial import MySerial

# 串口配置
SERIAL_PORT = "/dev/ttyS1"  # Windows端口，Linux下改为"/dev/ttyUSB0"
BAUDRATE = 115200

# 接收线程标志
running = True

def receive_data(serial):
    """
    接收数据的线程函数
    :param serial: 串口对象
    """
    global running
    print("开始接收数据...")
    
    while running:
        try:
            if serial.is_open and serial.ser is not None:
                # 尝试读取数据
                if serial.ser.in_waiting > 0:
                    data = serial.ser.readline().decode('utf-8').strip()
                    if data:
                        print(f"收到数据: {data}")
            time.sleep(0.1)
        except Exception as e:
            print(f"接收数据时出错: {str(e)}")
            time.sleep(0.1)

def main():
    global running
    
    # 初始化串口
    serial = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
    if not serial.open():
        print("串口打开失败，退出程序")
        return
    
    # 启动接收线程
    receive_thread = threading.Thread(target=receive_data, args=(serial,))
    receive_thread.daemon = True
    receive_thread.start()
    
    print("串口测试程序")
    print("==================================")
    print(f"当前串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print("==================================")
    print("输入要发送的数据，按Enter发送")
    print("输入'quit'退出程序")
    print("输入'test'发送测试数据")
    print("==================================")
    
    try:
        while True:
            # 读取用户输入
            user_input = input("请输入: ")
            
            if user_input.lower() == 'quit':
                print("退出程序...")
                break
            elif user_input.lower() == 'test':
                # 发送测试数据
                test_data = "DETA:1.2345,0.6789\n"
                success = serial.send_data(test_data)
                if success:
                    print(f"发送测试数据: {test_data.strip()}")
                else:
                    print("发送失败")
            else:
                # 发送用户输入的数据
                success = serial.send_data(user_input + "\n")
                if success:
                    print(f"发送数据: {user_input}")
                else:
                    print("发送失败")
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        # 停止接收线程
        running = False
        # 关闭串口
        serial.close()
        print("程序退出")

if __name__ == '__main__':
    main()