#!/usr/bin/env python

#import rospy
#from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

PIN = 18


def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(PIN, GPIO.IN)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    try:
        while True:
            time.sleep(1)
            on_off = not GPIO.input(PIN)
            # Toggle the output every second
            print(on_off)
            
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()




# def talker():
#     pub = rospy.Publisher('limit_switch', Bool, queue_size=10)
#     rospy.init_node('limit_switch', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
        
#         on_off = not GPIO.input(PIN)
#         pub.publish(on_off)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass