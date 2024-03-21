import numpy as np
import span6_to_tss1 as rs6


def rad_to_deg(rad):
    return rad*180/np.pi

data_span6 = r'.\data\20231213124021_svVexel_1312green2.span6'

with open(data_span6, 'rb') as f1:
    buffer = f1.read()

message_ids = []
offset = 0
loop = True
raw_imu_dict = {}

messages = []

data_span6 = r'.\data\20231213124021_svVexel_1312green2.span6'

with open(data_span6, 'rb') as f1:
    buffer = f1.read()

offset = 0
loop = True
messages = []
end = 0

hor_accel = 0.1
vert_accel = 0.1
heave = 0.0
roll = 0.0
pitch = 0.0

while loop:
    try:
        if len(buffer) == end:
            print('end of the file')
            loop = False

        header_dict, header_len = rs6.read_span6_header(buffer, offset)
        
        if header_len != 28 and header_len != 12:
            end = offset+header_len
            # print(header_dict["string"])
            messages.append(0)
        else:
            message_len = header_dict["message_length"]
            mid = header_dict['message_id']
            
            end = offset+header_len+message_len+4
            messages.append(mid)
            
            
            if mid == 1465:
                msg_dir = rs6.read_span6_message(buffer, offset, mid)
                hor_accel = msg_dir["up_vel"]/100
                vert_accel = np.sqrt(msg_dir["north_vel"]**2 + msg_dir["east_vel"]**2)/100
                roll = msg_dir["roll"]
                pitch = msg_dir["pitch"]
                    
        offset = end
        
        tss1 = rs6.create_tss1(hor_accel=hor_accel, vert_accel=vert_accel, heave=heave, roll=roll, pitch=pitch)
        
        print(tss1)
    except RuntimeError as a:
        print(a)
        loop = False