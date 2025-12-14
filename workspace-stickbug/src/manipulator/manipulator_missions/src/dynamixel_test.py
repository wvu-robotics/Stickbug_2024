
from dynamixel_lib import Dynamixel, U2D2
from dynamixel_lib import XM540W150, XL320, XL430W250, MX106
import time


'''
//==================================================================================== OVERVIEW ============================
// used as low level interface to linearly convert joint states to command values  following y = mx + b
// stepper joint ranges from 0 to 1.3 meters and has ~19685 steps per meter
// dynamixel joints  range from  0 to 4095 to represent 0 thourgh 2*PI radians
//
// motors are:
//  0) noncaptive stepper motor lift
//  1) shoulder dyanmixel 
//  2) elbow dynamixel
//  3) w1 on NOIS wrist
//  4) w2 on NOIS wrist
//  5) w3 on NOIS wrist
//  6) e1 gripper end-effector

// also publishes all the joint states of the manipulator
//=========================================================================== Global Variables ============================
'''

POSITION_CONTROL = 3
VELOCITY_CONTROL = 1

def bytes_to_int(byte_list):
    """
    Convert a list of bytes (little-endian format) to an integer.
    The first byte is the least significant byte.

    :param byte_list: List of bytes, where the first byte is the least significant.
    :return: The integer representation of the bytes.
    """
    result = 0
    for i, byte in enumerate(byte_list):
        result += byte * (256 ** i)
    return result

def bytes_to_twos_complement(byte_list):
    """
    Convert a list of bytes (little-endian format) to a signed integer using two's complement.
    The first byte is the least significant byte.

    :param byte_list: List of bytes, where the first byte is the least significant.
    :return: The signed integer representation of the bytes.
    """
    # Combine bytes into an unsigned integer
    unsigned_int = 0
    for i, byte in enumerate(byte_list):
        unsigned_int += byte * (256 ** i)

    # Convert to a signed integer using two's complement
    max_val = 2 ** 32  # Maximum value for 4 bytes
    if unsigned_int >= 2 ** 31:
        return unsigned_int - max_val
    else:
        return unsigned_int

# def test_motor(motor_id: int):
        
#         u2d2 = U2D2('/dev/ttyUSB0',57600)
#         motor = Dynamixel(XM540W150,motor_id,u2d2)
        
#         # must turn off torque to change modes
#         motor.write(XM540W150.TorqueEnable,0)
#         motor.write(XM540W150.OperatingMode,VELOCITY_CONTROL)
#         motor.write(XM540W150.TorqueEnable,1)
       
#         for x in range(5):
#             for n in range(300):
#                 motor.write(XM540W150.GoalVelocity,n)
        
#         motor.write(XM540W150.GoalVelocity,0)

#         motor.write(XM540W150.TorqueEnable,0)
#         motor.write(XM540W150.OperatingMode,POSITION_CONTROL)
#         motor.write(XM540W150.TorqueEnable,1)

#         motor.write(XM540W150.GoalPosition,2047)
#         time.sleep(1)

#         motor.write(XM540W150.TorqueEnable,0)

#test_motor(1)

u2d2 = U2D2('/dev/ttyUSB0',57600)

m1 = Dynamixel(XM540W150,1,u2d2)
m2 = Dynamixel(XM540W150,2,u2d2)
m3 = Dynamixel(XL430W250,3,u2d2)
m4 = Dynamixel(XL430W250,4,u2d2)
m5 = Dynamixel(XL320,5,u2d2)
m6 = Dynamixel(XL320,6,u2d2)
m7 = Dynamixel(XL320,7,u2d2)

m1.write(XM540W150.TorqueEnable,0)
m2.write(XM540W150.TorqueEnable,0)
m3.write(XL430W250.TorqueEnable,0)
m4.write(XL430W250.TorqueEnable,0)
m5.write(XL320.TorqueEnable,0)
m6.write(XL320.TorqueEnable,0)
m7.write(XL320.TorqueEnable,0)

# m1.write(XM540W150.OperatingMode,POSITION_CONTROL)
# m2.write(XM540W150.OperatingMode,POSITION_CONTROL)
# m3.write(XL430W250.OperatingMode,POSITION_CONTROL)
# m4.write(XL430W250.OperatingMode,POSITION_CONTROL)


# m1.write(XM540W150.TorqueEnable,1)
# m2.write(XM540W150.TorqueEnable,1)
# m3.write(XL430W250.TorqueEnable,1)
# m4.write(XL430W250.TorqueEnable,1)
# m5.write(XL320.TorqueEnable,1)
# m6.write(XL320.TorqueEnable,1)
# m7.write(XL320.TorqueEnable,1)

# m1.write(XM540W150.GoalPosition,2047)
# m2.write(XM540W150.GoalPosition,2047)
# m3.write(XL430W250.GoalPosition,2047)
# m4.write(XL430W250.GoalPosition,2047)
# m5.write(XL320.GoalPosition,2047)
# m6.write(XL320.GoalPosition,2047)
# m7.write(XL320.GoalPosition,2047)
# time.sleep(5)

# m1.write(XM540W150.TorqueEnable,0)
# m2.write(XM540W150.TorqueEnable,0)
# m3.write(XL430W250.TorqueEnable,0)
# m4.write(XL430W250.TorqueEnable,0)
# m5.write(XL320.TorqueEnable,0)
# m6.write(XL320.TorqueEnable,0)
# m7.write(XL320.TorqueEnable,0)

for t in range(100):
    var = m1.read(XM540W150.PresentVelocity)
    print(bytes_to_twos_complement(var[0]))
    time.sleep(.1)


