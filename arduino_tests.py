import serial
import time

ser = serial.Serial('/dev/ttyACM1')
while True:
    ser.write(b"relay-on\n")
    time.sleep(1)
    ser.write(b"relay-off\n")
    time.sleep(1)
