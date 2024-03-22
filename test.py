# Test COM
import serial
ser3 = serial.Serial('COM8', 115200, timeout=0.005)
if ser3.isOpen():
    print('Serial connection established!')
else:
    print('Error: Failed to establish serial connection.')
    
loop = True

i = 0

while True:
    print(ser3.readline())