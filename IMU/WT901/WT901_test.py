import serial
import struct
import time
last_print = time.time()
print_interval = 0.5  # 0.5초마다 출력

ser = serial.Serial('COM13', 9600, timeout=0.1)

packet = bytearray(11)
len_cnt = 0

# 센서 값 초기화
acc_x = acc_y = acc_z = acc_temp = 0.0
gyro_roll = gyro_pitch = gyro_yaw = gyro_temp = 0.0
ang_roll = ang_pitch = ang_yaw = ang_temp = 0.0
geomag_x = geomag_y = geomag_z = geomag_temp = 0.0

def sum2char(high, low):
    value = (high << 8) | low
    if value >= 32768:  # 2's complement
        value -= 65536
    return value

def parse_accel(data):
    global acc_x, acc_y, acc_z, acc_temp
    acc_x = sum2char(data[3], data[2]) / 32768 * 16
    acc_y = sum2char(data[5], data[4]) / 32768 * 16
    acc_z = sum2char(data[7], data[6]) / 32768 * 16
    acc_temp = sum2char(data[9], data[8]) / 100

def parse_gyro(data):
    global gyro_roll, gyro_pitch, gyro_yaw, gyro_temp
    gyro_roll = sum2char(data[3], data[2]) / 32768 * 2000
    gyro_pitch = sum2char(data[5], data[4]) / 32768 * 2000
    gyro_yaw = sum2char(data[7], data[6]) / 32768 * 2000
    gyro_temp = sum2char(data[9], data[8]) / 100

def parse_angle(data):
    global ang_roll, ang_pitch, ang_yaw, ang_temp
    ang_roll = sum2char(data[3], data[2]) / 32768 * 180
    ang_pitch = sum2char(data[5], data[4]) / 32768 * 180
    ang_yaw = sum2char(data[7], data[6]) / 32768 * 180
    ang_temp = sum2char(data[9], data[8]) / 100

def parse_geomag(data):
    global geomag_x, geomag_y, geomag_z, geomag_temp
    geomag_x = sum2char(data[3], data[2])
    geomag_y = sum2char(data[5], data[4])
    geomag_z = sum2char(data[7], data[6])
    geomag_temp = sum2char(data[9], data[8]) / 100

while True:
    if ser.in_waiting:
        byte = ser.read()[0]
        if len_cnt == 0 and byte == 0x55:
            packet[0] = byte
            len_cnt = 1
        elif len_cnt == 1:
            if 0x50 <= byte <= 0x58:
                packet[1] = byte
                len_cnt = 2
            else:
                len_cnt = 0
        elif 2 <= len_cnt < 11:
            packet[len_cnt] = byte
            len_cnt += 1
            if len_cnt == 11:
                order = packet[1]
                if order == 0x51:
                    parse_accel(packet)
                elif order == 0x52:
                    parse_gyro(packet)
                elif order == 0x53:
                    parse_angle(packet)
                    now = time.time()
                    if now - last_print >= print_interval:
                        print(f"Roll: {ang_roll:.2f}, Pitch: {ang_pitch:.2f}, Yaw: {ang_yaw:.2f}")
                        last_print = now
                elif order == 0x54:
                    parse_geomag(packet)
                len_cnt = 0
