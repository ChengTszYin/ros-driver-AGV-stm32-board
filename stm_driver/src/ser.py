import serial
import random
import time

def crc8(data,len):
    crc = 0
    for byte in range(len):
        crc = (crc + data[byte]) & 0xFF
    return crc & 0xFF

def send(a,b):
    datasSend = bytearray(8)
    datasSend[0] = 0x00
    datasSend[1] = 0x01
    datasSend[2] = (a >> 8) & 0xFF
    datasSend[3] = a & 0xFF
    datasSend[4] = 0X02
    datasSend[5] = (b >> 8) & 0xFF
    datasSend[6] = b & 0xFF
    datasSend[7] = crc8(datasSend[:7], 7)
    seri.write(datasSend)

seri = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
if seri.is_open:
    print("successful")
    while 1:
        _intput = input("speed: ")
        speed = int(_intput)
        print("swtich speed")
        send(speed, -speed)

else:
    print("failed")