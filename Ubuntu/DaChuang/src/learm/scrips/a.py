#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time

# Serial port configuration
PORT = '/dev/ttyUSB_Basedriver'      # Change to your port (COM3 on Windows)
BAUDRATE = 9600            # Match device baudrate
TIMEOUT = 1                # Timeout in seconds

# Data to send: 0x55 0x55 0x05 0x06 0x00 0x01
DATA_TO_SEND = bytearray([0x55, 0x55, 0x05, 0x06, 0x02, 0x01, 0x00])


def process_frame(frame):
    """
    处理接收到的数据帧
    """
    if len(frame) >= 4:
        hex_bytes = [frame[0], frame[1], frame[2], frame[3]]
        # rospy.loginfo("Received: 0x%02X 0x%02X 0x%02X 0x%02X", *hex_bytes)
        
        if hex_bytes[0] == 0x55 and hex_bytes[1] == 0x55:
            if hex_bytes[2] == 0x02 and hex_bytes[3] == 0x06:
                print("receved")
            elif hex_bytes[2] == 0x02 and hex_bytes[3] == 0x07:
                print("finish")

def main():
    # Open serial port
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT
    )
    print("Serial port {0} opened (baudrate {1})".format(PORT, BAUDRATE))

    # Send data
    ser.write(DATA_TO_SEND)
    hex_str = " ".join("0x{:02X}".format(byte) for byte in DATA_TO_SEND)
    print("Data sent: " + hex_str)

    buffer = bytearray()

    while True:
        len(buffer)

        # 使用缓冲区处理粘包问题
        data = ser.read(ser.in_waiting or 1)
        buffer.extend(data)

        # 查找完整帧（55 55开头）
        while len(buffer) >= 4:
            # 查找帧头
            found = False
            for i in range(len(buffer)-3):
                if buffer[i] == 0x55 and buffer[i+1] == 0x55:
                    # 提取4字节帧
                    frame = buffer[i:i+4]
                    buffer = buffer[i+4:]
                    process_frame(frame)
                    found = True
                    break
            if not found:
                break


if __name__ == "__main__":
    main()