import serial
import time

class MySerial:
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        初始化串口
        :param port: 串口端口，如 'COM1' 或 '/dev/ttyAMA0'
        :param baudrate: 波特率，默认115200
        :param timeout: 超时时间，默认1秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_open = False
    
    def open(self):
        """
        打开串口
        :return: 成功返回True，失败返回False
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.is_open = True
            print(f"串口 {self.port} 打开成功")
            return True
        except Exception as e:
            print(f"串口打开失败: {str(e)}")
            self.is_open = False
            return False
    
    def send_data(self, data):
        """
        发送数据
        :param data: 要发送的数据
        :return: 成功返回True，失败返回False
        """
        if not self.is_open or self.ser is None:
            print("串口未打开")
            return False
        
        try:
            self.ser.write(data.encode('utf-8'))
            return True
        except Exception as e:
            print(f"发送数据失败: {str(e)}")
            return False
    
    def send_deta(self, deta_x, deta_y):
        """
        发送偏差值
        :param deta_x: x方向偏差值
        :param deta_y: y方向偏差值
        :return: 成功返回True，失败返回False
        """
        # 格式化数据，例如: "DETA:1.2345,0.6789\n"
        data_str = f"DETA:{deta_x:.4f},{deta_y:.4f}\n"
        return self.send_data(data_str)
    
    def close(self):
        """
        关闭串口
        """
        if self.is_open and self.ser is not None:
            try:
                self.ser.close()
                self.is_open = False
                print(f"串口 {self.port} 关闭成功")
            except Exception as e:
                print(f"串口关闭失败: {str(e)}")
    
    def __del__(self):
        """
        析构函数，自动关闭串口
        """
        self.close()