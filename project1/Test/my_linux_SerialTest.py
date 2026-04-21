import serial

ser = serial.Serial("/dev/ttyS1", 115200, timeout=1)

print("开始接收")

while True:
    data = ser.readline()
    if data:
        print("收到:", data)