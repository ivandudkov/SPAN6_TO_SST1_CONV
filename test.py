import numpy as np
from span6_to_tss1 import create_tss1
import struct
import time
# Test COM
import serial
ser3 = serial.Serial('COM8', 115200, timeout=0.005)
if ser3.isOpen():
    print('Serial connection established!')
else:
    print('Error: Failed to establish serial connection.')
    
loop = True

# i = 0

while True:
    print(ser3.readline())
    
def decode_tss1(tss1_message):
    ### функция, чтобы проверить правильность генерации TSS1 (всё верно)
    hor_acc_lsb = 0.0383  # m/s^2
    vert_accel_lsb = 0.000625  # m/s^2
    hor_accel_b = (struct.unpack('B', bytes.fromhex(tss1_message[1:3]))[0])*hor_acc_lsb
    vert_accel_b = (struct.unpack('h', bytes.fromhex(tss1_message[3:8]))[0])*vert_accel_lsb
    
    print(tss1_message[1:3])
    print(tss1_message[3:8])
    print(f'horr: {hor_accel_b} vert: {vert_accel_b}')

def test_tss1():
    rng = np.random.default_rng()
    hor_accel = rng.random()*9.81
    vert_accel = rng.random()*20.47 - 20.47
    heave = rng.random()*99.99 - 99.99
    roll = rng.random()*99.99 - 99.99
    pitch = rng.random()*99.99 - 99.99
    print(f'{hor_accel} {vert_accel} {heave} {roll} {pitch}')
    tss1 = create_tss1(hor_accel=hor_accel, vert_accel=vert_accel, heave=heave, roll=roll, pitch=pitch)
    print(tss1)
    decode_tss1(tss1)