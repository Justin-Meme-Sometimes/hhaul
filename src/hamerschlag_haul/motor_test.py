import serial
import time

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
time.sleep(2)  # Arduino resets on serial connect

# Drive forward
ser.write(b'L150,R150\n')
time.sleep(2)

# Stop
ser.write(b'L0,R0\n')

ser.close()
