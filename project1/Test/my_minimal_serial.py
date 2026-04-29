#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最小串口通信代码
用于与云台（下位机）进行通信
"""

import time
import sys
import os

# 添加项目根目录到Python路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入MySerial模块
from Drivers.my_serial import MySerial


def main():
    """
    主函数
    """
    # 串口配置
    SERIAL_PORT = "/dev/ttyS1"  # Windows端口，Linux下改为"/dev/ttyUSB0"
    
    # 初始化串口
    serial_comm = MySerial(SERIAL_PORT)
    
    # 打开串口
    if not serial_comm.open():
        return
    
    try:
        # 循环发送数据
        print("开始循环发送数据...")
        print("按 Ctrl+C 停止发送")
        i = 0
        while True:
            deta_x = 0.01 * (i % 100)  # 0.00 到 0.99 循环
            deta_y = -0.01 * (i % 100)  # -0.00 到 -0.99 循环
            success = serial_comm.send_deta(deta_x, deta_y)
            if success:
                print(f"发送成功: DETAIL:{deta_x:.4f},{deta_y:.4f}")
            else:
                print("发送失败")
            i += 1
            time.sleep(0.02)  # 50Hz发送频率
        
    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        # 关闭串口
        serial_comm.close()


if __name__ == "__main__":
    main()
