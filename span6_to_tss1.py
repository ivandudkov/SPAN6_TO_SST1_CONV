import struct
from collections import namedtuple

import numpy as np

# TSS1 string example
# ':00FFCA -0003F-0325    0319'
# '<CR><LF> = 0x0D0x0A' # newline character

def create_tss1(hor_accel, vert_accel, heave, roll, pitch, status_f='F'):
   
    hor_accel_b = ''  # two byte hex
    hor_acc_lsb = 0.0383  # m/s^2
    vert_accel_b = ''  # two byte hex
    vert_accel_lsb = 0.000625  # m/s^2
    
    heave_b = int()  # four digit integer
    heave_b_polarity = ' '  # space if positive, minus sign (-) if negative
    
    status_flag = status_f  # F if INS Active, H if INS has not fompleted an aligment
    
    roll_b = int()  # four digit integer
    roll_b_polarity = ' '  # space if positive, minus sign (-) if negative
    
    pitch_b = int()  # four digit integer
    pitch_b_polarity = ' '  # space if positive, minus sign (-) if negative
    
    if hor_accel >= 9.771:
        hor_accel_b = struct.pack('B', int(hor_accel/hor_acc_lsb)).hex().upper()
    else:
        hor_accel_b = struct.pack('B', round(hor_accel/hor_acc_lsb)).hex().upper()
    
    vert_accel_b = struct.pack('h',round(vert_accel/vert_accel_lsb)).hex().upper()
    
    if heave < 0:
        heave_b_polarity = '-'
        heave_b = round(heave*-1*100)
    else:
        heave_b_polarity = ' '
        heave_b = round(heave*100)
    
    if roll < 0:
        roll_b_polarity = '-'
        roll_b = round(roll*-1*100)
    else:
        roll_b_polarity = ' '
        roll_b = round(roll*100)
    
    if pitch < 0:
        pitch_b_polarity = '-'
        pitch_b = round(pitch*-1*100)
    else:
        pitch_b_polarity = ' '
        pitch_b = round(pitch*100)
    
    
    tss1_message = f':{vert_accel_b}{hor_accel_b} {heave_b_polarity}{heave_b:04d}{status_flag}{roll_b_polarity}{roll_b:04d} {pitch_b_polarity}{pitch_b:04d}\n'
    
    return tss1_message

###########################
#####  Bin Datablock Reader
###########################
_ElementTypes = namedtuple(
    "_ElementTypes",
    [
        # These are the canonical names of low-level element types:
        "c8",
        "i8",
        "u8",  # 3 int types of 1 byte
        "i16",
        "u16",  # 2 int types of 2 bytes
        "i32",
        "u32",  # 2 int types of 4 bytes
        "i64",
        "u64",  # 2 int types of 8 bytes
        "f32",  # 1 float type of 4 bytes
        "f64",  # 1 float type of 4 bytes
    ],
)
elemT = _ElementTypes(**dict(zip(_ElementTypes._fields, _ElementTypes._fields)))


def elemD_(name: str, fmt: str, count=1):
    return (name, (fmt, count))

map_size_to_fmt = dict(
    (
        (elemT.c8, ("c", "B", 1)),
        (elemT.i8, ("b", "b", 1)),
        (elemT.u8, ("B", "u1", 1)),
        (elemT.i16, ("h", "i2", 2)),
        (elemT.u16, ("H", "u2", 2)),
        (elemT.i32, ("i", "i4", 4)),
        (elemT.u32, ("I", "u4", 4)),
        (elemT.i64, ("q", "i8", 8)),
        (elemT.u64, ("Q", "u8", 8)),
        (elemT.f32, ("f", "f4", 4)),
        (elemT.f64, ("d", "f8", 8)),
    )
)

class DataBlock:
    """
    Reads fixed-size blocks of structured binary data, according to a specified format
    """

    _byte_order_fmt = "<"

    def __init__(self, elements):
        self._sizes = self._util_take_sizes(elements)
        self._names = self._util_take_names(elements)
        self._struct = self._util_create_struct(self._sizes)
        self._np_types = self._util_create_np_types(self._names, self._sizes)

    @property
    def size(self):
        return self._struct.size

    @property
    def numpy_types(self):
        return self._np_types

    @staticmethod
    def _util_take_names(elements) -> tuple:
        return tuple(name for name, *_ in elements)

    @staticmethod
    def _util_take_sizes(elements) -> tuple:
        def f_take():
            for _, size in elements:
                if len(size) == 1:
                    size = size + (1,)
                yield size

        return tuple(f_take())

    @classmethod
    def _util_create_struct(cls, sizes) -> struct.Struct:
        fmts = [cls._byte_order_fmt]
        for type_name, count in sizes:
            count = "" if count == 1 else str(count)
            fmt, *_ = map_size_to_fmt[type_name]
            fmts.append(str(count) + fmt)
        return struct.Struct("".join(fmts))

    @classmethod
    def _util_create_np_types(cls, names, sizes):
        def f_name_fixer(idx, name):
            return f"__reserved{idx}__" if name is None else name

        bom = cls._byte_order_fmt
        types = []
        for idx, (name, (type_name, count)) in enumerate(zip(names, sizes)):
            name = f_name_fixer(idx, name)
            _, fmt, *_ = map_size_to_fmt[type_name]
            type_spec = [name, f"{bom}{fmt}"]
            if count > 1:
                type_spec += [(count,)]
            types.append(tuple(type_spec))
            del type_spec, fmt
            del idx, name, type_name, count
        return types

    @staticmethod
    def _util_gen_elements(fields, names):
        results = []
        name_index = 0
        for field in fields:
            part_a, part_b, *_ = field
            if part_a is None:
                results.append(tuple([None, part_b]))
            else:
                results.append(tuple([names[name_index], part_a]))
                name_index += 1
        return results
    
##############################
###  SPAN CPT Message Readers
##############################
header_long = DataBlock(
    (
        elemD_("sync1", elemT.c8),
        elemD_("sync2", elemT.c8),
        elemD_("sync3", elemT.c8),
        elemD_("header_length", elemT.u8),
        elemD_("message_id", elemT.u16),
        elemD_("message_type", elemT.c8),
        elemD_("port_adress", elemT.u8),
        elemD_("message_length", elemT.u16),
        elemD_("sequence", elemT.u16),
        elemD_("idle_time", elemT.u8),
        elemD_("time_status", elemT.u8),
        elemD_("week", elemT.u16),
        elemD_("ms", elemT.i32),
        elemD_("receiver_status", elemT.u32),
        elemD_("reserved", elemT.u16),
        elemD_("receiver_SW_version", elemT.u16)
    )
)

header_short = DataBlock(
    (
        elemD_("sync1", elemT.c8),
        elemD_("sync2", elemT.c8),
        elemD_("sync3", elemT.c8),
        elemD_("message_length", elemT.u8),
        elemD_("message_id", elemT.u16),
        elemD_("week_number", elemT.u16),
        elemD_("msec_from_week", elemT.u32)
    )
)

rawimusxb_message = DataBlock(  # ID 1462
    (
    elemD_("imu_info", elemT.c8),
    elemD_("imu_type", elemT.c8),
    elemD_("gnss_week", elemT.u16),
    elemD_("gnss_week_seconds", elemT.f64),
    elemD_("imu_status", elemT.u32),
    elemD_("z_accel", elemT.i32),
    elemD_("y_accel", elemT.i32),  # a negative value implies a positive
    # right-handed rotation about the y-axis marked on the IMU
    # a positive value implies a negative right-handed rotation
    #about the y-axis marked on the IMU
    elemD_("x_accel", elemT.i32),
    elemD_("z_gyro", elemT.i32),
    elemD_("y_gyro", elemT.i32),  # a negative value implies a positive
    # right-handed rotation about the y-axis marked on the IMU
    # a positive value implies a negative right-handed rotation
    #about the y-axis marked on the IMU
    elemD_("x_gyro", elemT.i32),
    elemD_("CRC", elemT.c8, 4),
    )
)

insattx_message = DataBlock(  # ID 1457
    (
        elemD_("ins_status", elemT.i32),
        elemD_("pos_type", elemT.i32),
        elemD_("roll", elemT.f64),
        elemD_("pitch", elemT.f64),
        elemD_("azimuth", elemT.f64),
        elemD_("roll_std", elemT.f32),
        elemD_("pitch_std", elemT.f32),
        elemD_("azimuth_std", elemT.f32),
        elemD_("exl_sol_stat", elemT.c8, 4),
        elemD_("time_sinse_upd", elemT.u16),
        elemD_("CRC", elemT.c8, 4)
    )
)

insatts_message = DataBlock(  # ID 319
    (
        elemD_("week", elemT.u32),
        elemD_("secs_into_week", elemT.f64),
        elemD_("roll", elemT.f64),
        elemD_("pitch", elemT.f64),
        elemD_("azimuth", elemT.f64),
        elemD_("status", elemT.i32),
        elemD_("CRC", elemT.c8, 4)
    )
)

inspvax_message = DataBlock(  # ID 1465
    (
        elemD_("ins_status", elemT.i32),
        elemD_("pos_type", elemT.i32),
        elemD_("lat", elemT.f64),
        elemD_("long", elemT.f64),
        elemD_("heihgt", elemT.f64),
        elemD_("undulation", elemT.f32),
        elemD_("north_vel", elemT.f64),
        elemD_("east_vel", elemT.f64),
        elemD_("up_vel", elemT.f64),
        elemD_("roll", elemT.f64),
        elemD_("pitch", elemT.f64),
        elemD_("azimuth", elemT.f64),
        elemD_("lat_std", elemT.f32),
        elemD_("long_std", elemT.f32),
        elemD_("height_std", elemT.f32),
        elemD_("north_vel_std", elemT.f32),
        elemD_("east_vel_std", elemT.f32),
        elemD_("up_vel_std", elemT.f32),
        elemD_("roll_std", elemT.f32),
        elemD_("pitch_std", elemT.f32),
        elemD_("azimuth_std", elemT.f32),
        elemD_("ext_sol_stat", elemT.c8, 4),
        elemD_("time_since_upd", elemT.u16),
        elemD_("CRC", elemT.c8, 4),
    )
)

inspvas_message = DataBlock(  # ID 508
    (
        elemD_("week", elemT.u32),
        elemD_("seconds", elemT.f64),
        elemD_("lat", elemT.f64),
        elemD_("long", elemT.f64),
        elemD_("heihgt", elemT.f64),
        elemD_("north_vel", elemT.f64),
        elemD_("east_vel", elemT.f64),
        elemD_("up_vel", elemT.f64),
        elemD_("roll", elemT.f64),
        elemD_("pitch", elemT.f64),
        elemD_("azimuth", elemT.f64),
        elemD_("status", elemT.i32),
        elemD_("CRC", elemT.c8, 4),
    )
) 

corrimudatas_message = DataBlock(  # ID 813
    (
        elemD_("week", elemT.u32),
        elemD_("seconds", elemT.f64),
        elemD_("pitch_rate", elemT.f64),
        elemD_("roll_rate", elemT.f64),
        elemD_("yaw_rate", elemT.f64),
        elemD_("lateral_acc", elemT.f64),
        elemD_("longitudinal_acc", elemT.f64),
        elemD_("vertical_acc", elemT.f64),
        elemD_("CRC", elemT.c8, 4),
    )
) 

syncheave_message = DataBlock(  # ID 1708
    (
        elemD_("heave", elemT.f64),
        elemD_("heave_std", elemT.f64),
        elemD_("CRC", elemT.c8, 4),
    )
) 

heave_message = DataBlock(  # ID 1382
    (
        elemD_("week", elemT.u32),
        elemD_("secs_into_week", elemT.f64),
        elemD_("heave", elemT.f64),
        elemD_("CRC", elemT.c8, 4),
    )
) 


messages_dict = {"1462": rawimusxb_message, "1457": insattx_message,
                 "319": insatts_message, "1465": inspvax_message,
                 "508": inspvas_message, "1708": syncheave_message,
                 "1382": heave_message, "813": corrimudatas_message}


long_start = bytes.fromhex('AA 44 12')
short_start = bytes.fromhex('AA 44 13')
ascii_usb_info = bytes.fromhex('3C 49 4E')

##############################

def check_header_type(buffer):
    
    if len(buffer) != 3:
        raise RuntimeError("Buffer's length is not 3 bytes!")
    
    header = 'UNKN'
    
    if long_start == buffer:
        header = 'LONG'
    elif short_start == buffer:
        header = 'SHORT'
    elif ascii_usb_info == buffer:
        header = 'INS_UPDATE'
        
    return header

def get_insupdate_size(buffer, offset):

    loop = True
    header_size = 0

    # Scan insupdate length
    while loop:
        try:
            buffer[offset + header_size:offset + header_size+1].decode(encoding='utf-8', errors='strict')
        except:
            loop = False
        else:
            header_size += 1
    
    return header_size


def read_span6_header(buffer, offset):
    header = check_header_type(buffer[offset:offset+3])
    header_dict = {}
    
    if header == "LONG":
        header_length = 28
        header_array = header_long._struct.unpack(buffer[offset:offset+header_long.size])
        for num, name in enumerate(header_long._names):
            header_dict[name] = header_array[num]
            
    elif header == "SHORT":
        header_length = 12
        header_array = header_short._struct.unpack(buffer[offset:offset+header_short.size])
        
        for num, name in enumerate(header_short._names):
            header_dict[name] = header_array[num]
            
    elif header == "INS_UPDATE":
        header_length = get_insupdate_size(buffer, offset)
        
        header_array = buffer[offset:offset+header_length]
        header_dict["string"] = header_array
            
    else:
        raise RuntimeError("Header is unknown or buffer is not at header start")
    
    return header_dict, header_length

def read_span6_message(buffer, offset, message_id, verbose=False):
    message = messages_dict[str(message_id)]
    message_array = message._struct.unpack(buffer[offset:offset+message.size])
    message_data = {}
    
    for num, name in enumerate(message._names):
        message_data[name] = message_array[num]
    
    
    if verbose:
        for num, name in enumerate(message._names):
            print(f'{name} : {message_array[num]}')
            
    return message_data
