import numpy as np
import serial

import span6_to_tss1 as rs6

data_span6 = r'.\data\20231213124021_svVexel_1312green2.span6'

if __name__ == '__main__':
    com_rx_buf = ''
    com_tx_buf = '' # send buffer
    
    port_list= rs6.get_com_list() # List of available serial ports
    
    print('Please, pick RX COM (that receives attitude data)')
    rx_com_name = rs6.pick_comport(port_list)
    rx_com_baud = rs6.pick_baud_rate()
    com_rx = rs6.serial_open(rx_com_name, rx_com_baud)
    
    print("Please, pick TX COM (that will transmit TSS1)")
    tx_com_name = rs6.pick_comport(port_list)
    tx_com_baud = rs6.pick_baud_rate()
    com_tx = rs6.serial_open(tx_com_name, tx_com_baud)
    
    
            
            
            
    # with open(data_span6, 'rb') as f1:
    #     buffer = f1.read()

    # offset = 0
    # loop = True
    # messages = []
    # end = 0

    # hor_accel = 1.0
    # vert_accel = 1.0
    # heave = 1.0
    # roll = 1.0
    # pitch = 1.0
    # num = 0
    # error_num = 0
    # with open(r'.\data\test.txt', 'w') as f2:

    #     while loop:
    #         try:
    #             if len(buffer) == offset:
    #                 print('end of the file')
    #                 loop = False

    #             header_dict, header_len = rs6.read_span6_header(buffer, offset)
                
    #             if header_len != 28 and header_len != 12:
    #                 end = offset+header_len
    #                 # print(header_dict["string"])
    #                 messages.append(0)
    #             else:
    #                 message_len = header_dict["message_length"]
    #                 mid = header_dict['message_id']
                    
    #                 end = offset+header_len+message_len+4
    #                 messages.append(mid)
                    
                    
    #                 if mid == 1465:

    #                     print(mid)
    #                     print(num)
    #                     num += 1
    #                     msg_dir = rs6.read_span6_message(buffer, offset, mid)
    #                     hor_accel = msg_dir["up_vel"]/100
    #                     vert_accel = np.sqrt(msg_dir["north_vel"]**2 + msg_dir["east_vel"]**2)/100
    #                     roll = msg_dir["roll"]
    #                     pitch = msg_dir["pitch"]
                            
    #             offset = end
                
    #             tss1 = rs6.create_tss1(hor_accel=hor_accel, vert_accel=vert_accel, heave=heave, roll=roll, pitch=pitch)
                
    #             f2.write(tss1)
    #         except RuntimeError as error:
    #             if mid == 1465:
    #                 print(f'error_num: {error_num}')
    #                 error_num += 1
    #             # print(mid)
    #             # print(msg_dir)
    #             print(error)
    #             loop = True