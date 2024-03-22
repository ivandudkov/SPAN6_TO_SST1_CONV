import numpy as np
import time

import span6_to_tss1 as rs6



if __name__ == '__main__':
  
    port_list= rs6.get_com_list() # List of available serial ports
    
    print('Please, pick RX COM (that receives attitude data)')
    rx_com_name = rs6.pick_comport(port_list)
    rx_com_baud = rs6.pick_baud_rate()
    com_rx = rs6.serial_open(rx_com_name, rx_com_baud)
    
    print("Please, pick TX COM (that will transmit TSS1)")
    tx_com_name = rs6.pick_comport(port_list)
    tx_com_baud = rs6.pick_baud_rate()
    com_tx = rs6.serial_open(tx_com_name, tx_com_baud)
                
    
    loop = True
    hor_accel = 0
    vert_accel = 0
    heave = 0
    roll = 0
    pitch = 0
    rx_buf = ''
    
    while loop:
        try:
            rx_buf = ''
            rx_buf = com_rx.read()
            if rx_buf != b'':
                time.sleep(0.005)
                rx_buf = rx_buf + com_rx.read_all()
                
                while 3 > len(rx_buf):
                    rx_buf = rx_buf + com_rx.read(3 - len(rx_buf))
                header, offset = rs6.find_header(rx_buf)
                
                if header == "SHORT" or header == "LONG":
                    print(header)
                    while offset + 28 > len(rx_buf):
                        rx_buf = rx_buf + com_rx.read(offset + 28 - len(rx_buf))
                        
                    header_dict, header_len = rs6.read_span6_header(rx_buf, header, offset)
                    message_start = offset + header_len
                    
                    message_len = header_dict["message_length"]
                    message_id = header_dict['message_id']
                    
                    while message_start + message_len + 4 > len(rx_buf):
                        rx_buf = rx_buf + com_rx.read(message_start + message_len + 4 - len(rx_buf))
                    
                    if message_id == 1465:
                        msg_dir = rs6.read_span6_message(rx_buf, message_start, message_id)
                        
                        hor_accel = msg_dir["up_vel"]/200
                        vert_accel = np.sqrt(msg_dir["north_vel"]**2 + msg_dir["east_vel"]**2)/200
                        roll = msg_dir["roll"]
                        pitch = msg_dir["pitch"]
                        
                    elif message_id == 1708:
                        msg_dir = rs6.read_span6_message(rx_buf, message_start, message_id)
                        
                        hor_accel = msg_dir["vertical_acc"]*200
                        vert_accel = np.sqrt(msg_dir["lateral_acc"]**2 + msg_dir["longitudinal_acc"]**2)*200
                        roll = msg_dir["roll_rate"]*200
                        pitch = msg_dir["pitch_rate"]*200
                        
                    elif message_id == 813:
                        msg_dir = rs6.read_span6_message(rx_buf, message_start, message_id)
                        heave = msg_dir["heave"]
                    
                    tss1 = rs6.create_tss1(hor_accel=hor_accel, vert_accel=vert_accel, heave=heave, roll=roll, pitch=pitch)
                    com_tx.write(tss1.encode('utf-8'))
                    print(tss1)

            time.sleep(0.005)
        except RuntimeError as error:
            print(error)
            # tss1 = rs6.create_tss1(hor_accel=hor_accel, vert_accel=vert_accel, heave=heave, roll=roll, pitch=pitch)
            # print(tss1)
        
    
    # with open(data_span6, 'rb') as f1:
    #     buf = f1.read()
        
    # com_tx.write(buf[0:10000])
    # rs6.serial_close(com_tx)
    # i = 0
    # while i < 10:
    #     com_tx.write(buf)
    #     i+= 1 

