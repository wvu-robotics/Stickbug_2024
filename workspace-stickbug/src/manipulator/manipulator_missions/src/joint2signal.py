import numpy as np
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Int64
import serial
import os
import yaml

from dynamixel_lib import Dynamixel, U2D2
from dynamixel_lib import XM540W150, XL320, XL430W250, MX106
import time
from std_msgs.msg import Int64

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
//  3) w1 on sphere wrist
//  4) w2 on sphere wrist
//  5) e1 on poll end-effector
//  6) e2 on poll end-effector
//  7) e3 on poll end-effector

// also publishes all the joint states of the manipulator
//=========================================================================== Global Variables ============================
'''
# Initialize variables
#u2d2 = U2D2('/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WI4C5-if00-port0',57600)
u2d2_port = '/dev/ttyUSB0'
u2d2 = None
POSITION_CONTROL = 3
VELOCITY_CONTROL = 1
JOINT_MODE = 2
WHEEL_MODE = 1

arm_state = 0

# arm states 
ALL_FLOWERS_IN_COLLISION = 1 # red
GOING_TO_FLOWER = 2 # green
POLLINATING = 3 # yellow
BACKING_UP = 4  # blue
GOING_TO_TOLD_POSE = 5 # pink
NO_FLOWERS = 6 # blue-green
NO_FLOWERS_WITHIN_REACH = 7 # white

is_position_ctr = False
goal_joints = JointState()
actual_joints = JointState()
ee_joints = JointState()
got_ee_joints = False

got_goal = False

position_fits = []
velocity_fits = []

arduino_port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
arduino_baud_rate = 9600
arduino_ser = None

stepper_count = None
stepper_position = None
stepper_delay = 200 # microseconds
stepper_online = False

#----------------------------------------------- LINEAR FIT CLASS ------------------------------------------------------------
class LinearJointFit:
    def __init__(self, min_signal, max_signal, min_joint, max_joint):
        self.min_signal = min_signal
        self.max_signal = max_signal
        self.min_joint = min_joint
        self.max_joint = max_joint

    def joint2signal(self, joint):
        # Convert joint position to signal value
        scale = (self.max_signal - self.min_signal) / (self.max_joint - self.min_joint)
        signal = scale * (joint - self.min_joint) + self.min_signal
        # Clip the output to be within max and min signals
        return max(self.min_signal, min(self.max_signal, signal))

    def signal2joint(self, signal):
        # Convert signal value to joint position
        scale = (self.max_joint - self.min_joint) / (self.max_signal - self.min_signal)
        joint = scale * (signal - self.min_signal) + self.min_joint
        # Clip the output to be within max and min joints
        return max(self.min_joint, min(self.max_joint, joint))

def init_fits():
    global position_fits, velocity_fits

    steps_per_m = 50000/.32
    max_position = 1.3
    max_step = steps_per_m*max_position

    yaw_max = 80*math.pi/180

    position_fits = [LinearJointFit(0,max_step,0,max_position),               # stepper
                     LinearJointFit(1023,3073,-math.pi/2,math.pi/2),          # shoulder
                     LinearJointFit(1023,3073,-math.pi/2,math.pi/2),          # elbow
                     LinearJointFit(1137,2957,-yaw_max, yaw_max),             # w1 -> yaw
                     LinearJointFit(1023,3073,-math.pi/2,math.pi/2),          # w2 -> pitch
                     LinearJointFit(204,820,-math.pi/2,math.pi/2),            # e1
                     LinearJointFit(204,820,-math.pi/2,math.pi/2),            # e2
                     LinearJointFit(204,820,-math.pi/2,math.pi/2)]            # e3
    
    rpm_sig = .229
    rad_s_sig = rpm_sig*2*math.pi/60 # convert to rad/s per signal

    small_rpm_sig = .111
    small_rad_s_sig = small_rpm_sig*2*math.pi/60 # convert to rad/s per signal

    max_lift_speed = (.32/50000)*(1/(2*200e-6)) # .32 meters per 50000 steps, 200 microseconds per half pulse (1 pulse = 1 step) 

    velocity_fits = [LinearJointFit(-max_lift_speed,max_lift_speed,-max_lift_speed,max_lift_speed), #need to fix
                     LinearJointFit(-230, 230, -230*rad_s_sig, 230*rad_s_sig),
                     LinearJointFit(-230, 230, -230*rad_s_sig, 230*rad_s_sig),
                     LinearJointFit(-250, 250, -250*rad_s_sig, 250*rad_s_sig),
                     LinearJointFit(-250, 250, -250*rad_s_sig, 250*rad_s_sig),
                     LinearJointFit(204,820,-math.pi/2,math.pi/2),
                     LinearJointFit(204,820,-math.pi/2,math.pi/2),
                     LinearJointFit(204,820,-math.pi/2,math.pi/2)]

#----------------------------------------------- DYNAMIXEL HELPER FUNCTIONS ---------------------------------------------------

def init_dyna_motors():
    global dyna_motors, u2d2
    dyna_motors = [Dynamixel(XM540W150,1,u2d2),
                   Dynamixel(XM540W150,2,u2d2),
                   Dynamixel(XL430W250,3,u2d2),
                   Dynamixel(XL430W250,4,u2d2),
                   Dynamixel(XL320,5,u2d2),
                   Dynamixel(XL320,6,u2d2),
                   Dynamixel(XL320,7,u2d2)]

def dynamixels_torque_on(on):
    global dyna_motors
    if on:
        dyna_motors[0].write(XM540W150.TorqueEnable,1)
        dyna_motors[1].write(XM540W150.TorqueEnable,1)
        dyna_motors[2].write(XL430W250.TorqueEnable,1)
        dyna_motors[3].write(XL430W250.TorqueEnable,1)
        dyna_motors[4].write(XL320.TorqueEnable,1)
        dyna_motors[5].write(XL320.TorqueEnable,1)
        dyna_motors[6].write(XL320.TorqueEnable,1)
    else:
        dyna_motors[0].write(XM540W150.TorqueEnable,0)
        dyna_motors[1].write(XM540W150.TorqueEnable,0)
        dyna_motors[2].write(XL430W250.TorqueEnable,0)
        dyna_motors[3].write(XL430W250.TorqueEnable,0)
        dyna_motors[4].write(XL320.TorqueEnable,0)
        dyna_motors[5].write(XL320.TorqueEnable,0)
        dyna_motors[6].write(XL320.TorqueEnable,0)

def dynamixels_set_position_control():
    global dyna_motors, POSITION_CONTROL, JOINT_MODE

    dynamixels_torque_on(0)
    
    dyna_motors[0].write(XM540W150.OperatingMode,POSITION_CONTROL)
    dyna_motors[1].write(XM540W150.OperatingMode,POSITION_CONTROL)
    dyna_motors[2].write(XL430W250.OperatingMode,POSITION_CONTROL)
    dyna_motors[3].write(XL430W250.OperatingMode,POSITION_CONTROL)
    dyna_motors[4].write(XL320.ControlMode,JOINT_MODE)
    dyna_motors[5].write(XL320.ControlMode,JOINT_MODE)
    dyna_motors[6].write(XL320.ControlMode,JOINT_MODE)
    
    dynamixels_torque_on(1)


def dynamixels_set_velocity_control():
    global dyna_motors, VELOCITY_CONTROL

    dynamixels_torque_on(0)

    dyna_motors[0].write(XM540W150.OperatingMode,VELOCITY_CONTROL)
    dyna_motors[1].write(XM540W150.OperatingMode,VELOCITY_CONTROL)
    dyna_motors[2].write(XL430W250.OperatingMode,VELOCITY_CONTROL)
    dyna_motors[3].write(XL430W250.OperatingMode,VELOCITY_CONTROL)
    dyna_motors[4].write(XL320.ControlMode,JOINT_MODE)
    dyna_motors[5].write(XL320.ControlMode,JOINT_MODE)
    dyna_motors[6].write(XL320.ControlMode,JOINT_MODE)

    dynamixels_torque_on(1)

def dynamixels_set_goal_position(goal_signals):
    global dyna_motors
    
    dyna_motors[0].write(XM540W150.GoalPosition, int(goal_signals[1]))
    time.sleep(.001)
    dyna_motors[1].write(XM540W150.GoalPosition, int(goal_signals[2]))
    time.sleep(.001)
    dyna_motors[2].write(XL430W250.GoalPosition, int(2957-goal_signals[3]+1137))
    time.sleep(.001)
    dyna_motors[3].write(XL430W250.GoalPosition, int(goal_signals[4]))
    time.sleep(.001)
    dyna_motors[4].write(XL320.GoalPosition, int(goal_signals[5]))
    time.sleep(.001)
    dyna_motors[5].write(XL320.GoalPosition, int(goal_signals[6]))
    time.sleep(.001)
    dyna_motors[6].write(XL320.GoalPosition, int(goal_signals[7]))
    time.sleep(.001)

def dynamixels_set_goal_velocity(goal_signals):
    global dyna_motors, actual_joints, position_fits

    adjusted_sig = check_velocity_commands(goal_signals)
    
    goal_signals[:5] = adjusted_sig[:5]

    # skip stepper mode---------------------------------------------- 
    dyna_motors[0].write(XM540W150.GoalVelocity, int(goal_signals[1]))
    dyna_motors[1].write(XM540W150.GoalVelocity, int(goal_signals[2]))
    dyna_motors[2].write(XL430W250.GoalVelocity, int(goal_signals[3]))
    dyna_motors[3].write(XL430W250.GoalVelocity, int(goal_signals[4]))
    # end effector does not have velocity control so leave it in position control
    dyna_motors[4].write(XL320.GoalPosition, int(goal_signals[5]))
    dyna_motors[5].write(XL320.GoalPosition, int(goal_signals[6]))
    dyna_motors[6].write(XL320.GoalPosition, int(goal_signals[7]))

def dynamixels_set_leds():
    global dyna_motors, arm_state
    dyna_motors[4].write(XL320.LED,arm_state)
    dyna_motors[5].write(XL320.LED,arm_state)
    dyna_motors[6].write(XL320.LED,arm_state)

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

#------------------------------------------------ STEPPER HELPER FUNCTIONS
def send_steps_to_arduino(delta_steps):
    global arduino_ser, stepper_count, stepper_delay, stepper_online
    if arduino_ser is not None and stepper_online:
        # Combine delta_steps and stepper_delay into a single string, separated by a delimiter
        command = f"{int(delta_steps)},{int(stepper_delay)}\n"
        arduino_ser.write(command.encode())
        #arduino_ser.write(f"{int(delta_steps)}\n".encode())
        print(int(delta_steps))
        response = arduino_ser.readline().decode().strip()
        stepper_count += delta_steps
        if response:
            #print(f"Arduino response: {response}")
            save_stepper_position(stepper_count)
        else:
            print("No response from Arduino or timeout occurred.")
    else:
        print("Serial connection not established.")

def close_serial_connection():
    global arduino_ser
    if arduino_ser is not None:
        arduino_ser.close()

def init_stepper_motor():
    setup_serial_connection()
    print("Initialized stepper")
    

def setup_serial_connection():
    global arduino_ser, arduino_port, arduino_baud_rate
    arduino_ser = serial.Serial(arduino_port, arduino_baud_rate, timeout=5)
    time.sleep(2)  # Initial delay to allow Arduino to reset

def save_stepper_position(stepper_count):
    try:
        current_namespace = rospy.get_namespace()
        # Assuming the namespace format is "stickbug/arm#/...", extract the arm number
        arm_number = current_namespace.split('/')[2]  # This assumes the format "stickbug/arm#/"
        
        # Use the arm number to construct the YAML file path dynamically
        yaml_file_path = os.path.expanduser(f'~/workspace-stickbug/src/manipulator/manipulator_missions/config/{arm_number}.yaml')


        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file) or {}

        config['stepper_position'] = stepper_count

        with open(yaml_file_path, 'w') as file:
            yaml.safe_dump(config, file)

        #print("Stepper position saved for ", arm_number)
    except Exception as e:
        print(f"Failed to save stepper position: {e}")

def calc_delay(velocity):
    global stepper_delay
    stepper_delay = int((.32/50000)*(1/(2*velocity))*1e6)
    if stepper_delay < 200:
        stepper_delay = 200

def calc_velocity():
    global stepper_delay
    return int((.32/50000)*(1/(2*stepper_delay)))

def calc_delta_steps():
    global stepper_delay
    return 2*(.1*1e6)/(2*stepper_delay)

#------------------------------------------  CALCULATION HELPER FUNCTIONS ------------------------------------------------

def calculate_goal_signals():
    global goal_joints
    goal_signals = [0,0,0,0,0,0,0,0]

    if is_position_ctr:
        for i in range(len(position_fits)):
            goal_signals[i] = position_fits[i].joint2signal(goal_joints.position[i])
    else:
        for i in range(len(velocity_fits)):
            goal_signals[i] = velocity_fits[i].joint2signal(goal_joints.velocity[i])
            if i == 3:
                goal_signals[i] = velocity_fits[i].joint2signal(-goal_joints.velocity[i])




    return goal_signals

def calculate_actual_joints():
    global actual_joints
 
    actual_joints.header.stamp = rospy.Time.now()

    pos_sig = read_position_signal()
    vel_sig = read_velocity_signal()


    for i in range(len(position_fits)):
        actual_joints.position[i] = position_fits[i].signal2joint(pos_sig[i])
        actual_joints.velocity[i] = velocity_fits[i].signal2joint(vel_sig[i])
    

def read_position_signal():
    global stepper_count
    signals = [0,0,0,0,0,0,0,0]
    signals[0] = stepper_count
    signals[1] = bytes_to_int(dyna_motors[0].read(XM540W150.PresentPosition)[0])
    signals[2] = bytes_to_int(dyna_motors[1].read(XM540W150.PresentPosition)[0])
    signals[3] = bytes_to_int(dyna_motors[2].read(XL430W250.PresentPosition)[0])
    signals[4] = bytes_to_int(dyna_motors[3].read(XL430W250.PresentPosition)[0])
    signals[5] = bytes_to_int(dyna_motors[4].read(XL320.PresentPosition)[0])
    signals[6] = bytes_to_int(dyna_motors[5].read(XL320.PresentPosition)[0])
    signals[7] = bytes_to_int(dyna_motors[6].read(XL320.PresentPosition)[0])

    
    signals[3] = 2957 - signals[3] + 1137

    return signals

def read_velocity_signal():
    signals = [0,0,0,0,0,0,0,0]
    signals[0] = 0
    signals[1] = bytes_to_twos_complement(dyna_motors[0].read(XM540W150.PresentVelocity)[0])
    signals[2] = bytes_to_twos_complement(dyna_motors[1].read(XM540W150.PresentVelocity)[0])
    signals[3] = bytes_to_twos_complement(dyna_motors[2].read(XL430W250.PresentVelocity)[0])
    signals[4] = bytes_to_twos_complement(dyna_motors[3].read(XL430W250.PresentVelocity)[0])
    signals[5] = bytes_to_twos_complement(dyna_motors[4].read(XL320.PresentSpeed)[0])
    signals[6] = bytes_to_twos_complement(dyna_motors[5].read(XL320.PresentSpeed)[0])
    signals[7] = bytes_to_twos_complement(dyna_motors[6].read(XL320.PresentSpeed)[0])

    signals[3] = -signals[3]

    return signals

def check_velocity_commands(velocity_commands):
    global dyna_motors, actual_joints, position_fits, velocity_fits
    """
    Check if the velocity commands are valid given the current joint positions.

    :param velocity_commands: List of velocity commands for each joint.
    :param actual_joints: JointState object containing the current joint positions.
    :param position_fits: List of LinearJointFit objects for each joint.
    :return: List of adjusted velocity commands.
    """
    adjusted_velocities = []
    for i, velocity in enumerate(velocity_commands):
        
        position_fit = position_fits[i]
        velocity_fit = velocity_fits[i]
        current_position = actual_joints.position[i]
        
        adjusted_velocity = velocity
        
        if i != 3:
            # set maximum speed ---------------------------------------------
            if velocity_fit.signal2joint(adjusted_velocity) > 0.1:
                adjusted_velocity = velocity_fit.joint2signal(.1)
                print("maxed out velocity!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            # set minimum speed ----------------------------------------------
            if velocity_fit.signal2joint(adjusted_velocity) < -0.1:
                adjusted_velocity = velocity_fit.joint2signal(-0.1)
                print("min out velocity????????????????????????????????????")

            # Check minimum joint limit---------------------------------------
            if current_position <= position_fit.min_joint and adjusted_velocity < 0:
                adjusted_velocity = 0
                print("motor")
                print(i)
                print("AT POSITION MINIMUM ===============================")
            
            # Check maximum joint limit-------------------------------------
            if current_position >= position_fit.max_joint and adjusted_velocity > 0:
                adjusted_velocity = 0
                print("motor")
                print(i)
                print("AT POSITION MAXIMUM ===============================")
        else:
            # set maximum speed ---------------------------------------------
            if velocity_fit.signal2joint(adjusted_velocity) > 0.1:
                adjusted_velocity = velocity_fit.joint2signal(.1)
                print("maxed out velocity!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            # set minimum speed ----------------------------------------------
            if velocity_fit.signal2joint(adjusted_velocity) < -0.1:
                adjusted_velocity = velocity_fit.joint2signal(-0.1)
                print("min out velocity????????????????????????????????????")

            # Check minimum joint limit---------------------------------------
            if current_position <= position_fit.min_joint and adjusted_velocity > 0:
                adjusted_velocity = 0
                print("motor")
                print(i)
                print("AT POSITION MINIMUM ===============================")
            
            # Check maximum joint limit-------------------------------------
            if current_position >= position_fit.max_joint and adjusted_velocity < 0:
                adjusted_velocity = 0
                print("motor")
                print(i)
                print("AT POSITION MAXIMUM ===============================")
        
        
        
        adjusted_velocities.append(adjusted_velocity)


    return adjusted_velocities
#---------------------------------------------------- CALLBACKS ------------------------------------------------------

def goal_joints_cb(joint_msg):
    global goal_joints, got_goal
    goal_joints = joint_msg
    got_goal = True

def torque_cb(bool_msg):
    dynamixels_torque_on(on=bool_msg.data)

def pos_ctr_cb(bool_msg):
    global is_position_ctr
    is_position_ctr = bool_msg.data
    if is_position_ctr:
        dynamixels_set_position_control()
    else:
        dynamixels_set_velocity_control()

def arm_state_cb(int_msg):
    global arm_state
    arm_state = int_msg.data
    dynamixels_set_leds()

def ee_joints_cb(joint_msg):
    global ee_joints, got_ee_joints
    ee_joints = joint_msg
    got_ee_joints = True
    
    
#--------------------------------------------------- MAIN -------------------------------------------------------------

def joint2signal():

    global is_position_ctr, actual_joints , got_goal, u2d2, u2d2_port, stepper_count, got_ee_joints
    global stepper_online
    

    # initialize node ----------------------------------------------------------
    rospy.init_node('joint2signal', anonymous=True)
    rate = rospy.Rate(100)

    # publisher and subscribers ------------------------------------------------
    rospy.Subscriber("manipulator_goal_joints", JointState, goal_joints_cb)
    rospy.Subscriber("torque_on", Bool, torque_cb)
    rospy.Subscriber("position_control", Bool, pos_ctr_cb)
    rospy.Subscriber("arm_state", Int64, arm_state_cb)
    rospy.Subscriber("ee_joints", JointState, ee_joints_cb)
    

    #
    actual_joint_pub = rospy.Publisher("/stickbug/joint_states", JointState, queue_size=10)
    actual_joint2_pub = rospy.Publisher("manipulator_actual_joints", JointState, queue_size=10)

    # rosparameters ------------------------------------------------------------------- 
    # Dynamically get the current namespace
    current_namespace = rospy.get_namespace()

    # Construct the parameter name dynamically
    param_name_u2d2 = current_namespace + "u2d2"  # Assuming 'u2d2' is the parameter you want to access
    # Get a parameter, including its namespace, with a default value if the parameter is not found
    param_val = rospy.get_param(param_name_u2d2, u2d2_port)
    if param_val  == u2d2_port:
        rospy.logwarn(f"The default value for 'u2d2' is being used: {u2d2_port}")
    u2d2_port = param_val
    print('Parameter value:',  u2d2_port )

    param_name_step_count = current_namespace + "stepper_position"
    param_val = rospy.get_param(param_name_step_count, stepper_count)
    if param_val  == stepper_count:
        rospy.logwarn(f"The default value for 'stepper_count' is being used: {stepper_count}")
    stepper_count = param_val
    print('starting stepper position:',  stepper_count )
    stepper_online = True

    # Initialize ------------------------------------------------------------------------
    u2d2 = U2D2(u2d2_port,57600)
    init_dyna_motors()
    init_stepper_motor()
    dynamixels_set_velocity_control()
    dynamixels_torque_on(True)
    init_fits()

    if current_namespace[0] == '/':
        ns = current_namespace[1:]
    else:
        ns = current_namespace
    
    #TODO make dynamic
    actual_joints.name = [ns+'slider', ns+'shoulder', ns+'elbow', ns+'w1',ns+ 'w2', ns+'e1', ns+'e2', ns+'e3'] 
    actual_joints.position = [0,0,0,0,0,0,0,0]
    actual_joints.velocity = [0,0,0,0,0,0,0,0]
    calculate_actual_joints()
    
    
    
    # main loop -----------------------------------------------------------------
    while not rospy.is_shutdown():
        
        # freeze the arm if we are pollinating
        if arm_state == POLLINATING and got_ee_joints:
            if not is_position_ctr:
                e1_sig = velocity_fits[5].joint2signal(ee_joints.position[0])
                e2_sig = velocity_fits[6].joint2signal(ee_joints.position[1])
                e3_sig = velocity_fits[7].joint2signal(ee_joints.position[2])
                dynamixels_set_goal_velocity([0, 0,0,0,0, e1_sig, e2_sig, e3_sig])
            else:
                # TODO FIGURE OUT POSITION CONTROL
                pass
            
        
        # if we recieved goal joints send command to the motors
        elif got_goal:
            goal_signals = calculate_goal_signals()
            if is_position_ctr:
                send_steps_to_arduino(int(goal_signals[0] - stepper_count))
                dynamixels_set_goal_position(goal_signals)
             
                print(goal_signals)

            else:
                dynamixels_set_goal_velocity(goal_signals)
                stepper_velocity = goal_joints.velocity[0]
                stepper_position = goal_joints.position[0]
                goal_steps = position_fits[0].joint2signal(stepper_position)
                if abs(stepper_velocity) > 1e-6:
                    calc_delay(abs(stepper_velocity))
                    send_steps_to_arduino(int(goal_steps - stepper_count))

            
            got_goal = False # reset flag so we dont continuously write to the dynamixel


        # publish messages 
        calculate_actual_joints()
        actual_joint_pub.publish(actual_joints)
        actual_joint2_pub.publish(actual_joints)
           
        
        # continue to loop -------------------------------------------------------
        rate.sleep()
    

if __name__ == '__main__':
    try:
      joint2signal()
    except rospy.ROSInterruptException:
       dynamixels_set_goal_velocity([0,0,0,0,0,0,0,0])
       pass
