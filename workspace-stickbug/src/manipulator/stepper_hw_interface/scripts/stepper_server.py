#!/usr/bin/env python3

from __future__ import print_function

from stepper_hw_interface.srv import stepper_move,stepper_moveResponse
import rospy
import json
import rospkg

from time import sleep
import RPi.GPIO as GPIO

# ============================================= Overview ====================================
#
# absolute position control service for stepper motor interface for raspberry pi 4
#
#   1. loads in last step position of the stepper motor and sets to current position
#   2. waits for service request to move to a goal step position
#   3. moves to goal step position and saves the current position


# ============================================= Global Variables =============================

DIR = 15      #  GPIO pin 15 on pi is direction
STEP = 14     #  GPIO pin 14 on pi is the step pulse

CCW = True    # set counter clockwise pin value for pi
CW = False    # set clockwise pin value for pi

goal_step = 0 # target goal location     [steps]
delay = .001  # time delay between steps [sec]

# ============================================= Initalization ================================

# load in last position and fill curr_step -----------------
rospack = rospkg.RosPack()
try:
    curr_step = json.load(open(rospack.get_path("stepper_hw_interface")+"/config/start_position.txt",'r'))
except IOError:
    curr_step = 0
    json.dump(curr_step,open(rospack.get_path("stepper_hw_interface")+"/config/start_position.txt",'w'))


#   set gpio pin modes on the pi --------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(STEP,GPIO.OUT)

# ======================================== FUNCTIONS ======================================

# helper function to step a direction, using stepper driver interface on pi
def step(direction, steps):

    # set direction
    GPIO.output(DIR,direction)

    # do the steps
    for x in range(steps):
        GPIO.output(STEP,GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP,GPIO.LOW)
        sleep(delay) 

# stepper service that moves the stepper motor to the goal location
def handle_stepper(req):
    global curr_step
    goal_step = req.goal_step
    
    # move the motor
    if curr_step < goal_step :
        step(CW, abs(goal_step-curr_step))
    elif curr_step > goal_step :
        step(CCW, abs(goal_step-curr_step))
   
    curr_step = goal_step
    # save the current position to the text file
    json.dump(curr_step,open(rospack.get_path("stepper_hw_interface")+"/config/start_position.txt",'w'))

    return stepper_moveResponse(curr_step)

# ========================================= MAIN ====================================================
def stepper_server():
    
    # create node
    rospy.init_node('stepper_server')
    ns = rospy.get_namespace()

    # ros parameters -------------------------------------------------------
    DIR = rospy.get_param(ns + "/DIR", DIR)
    STEP = rospy.get_param(ns + "/STEP", STEP)
    CCW = rospy.get_param(ns + "/CCW", CCW)
    CW = rospy.get_param(ns + "/CW", CW)
    delay = rospy.get_param(ns + "/delay", delay)
    # initalize ------------------------------------------------------------

    # create server
    s = rospy.Service('move_stepper', stepper_move, handle_stepper)
    
    # ros continue
    rospy.spin()

if __name__ == "__main__":
    stepper_server()
