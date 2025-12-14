#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition 

class xl320_Test_Node(Node):
    
    def __init__(self):
        super().__init__("xl320_test")
        self.counter_ = 0
        self.plan = 0
        self.posX = 0
        self.posY = 0
        self.S_H_L = 18
        self.LL = 44.5
        self.cmd_pos_pub_ = self.create_publisher(SetPosition, "/set_position", 10)
        self.timer_ = self.create_timer(0.05, self.motion_planner)
        self.get_logger().info("XL 320 Test node has started")

    def motion_planner(self):
        if self.plan == 0:
            pass
            if self.counter_ >= 6.283:
                self.counter_ = 1.571
                self.plan = 1
            else:
                self.posX = 0
                self.posY = 0.4*math.sin(self.counter_)
                self.counter_ += 0.04
                self.send_position_command()
        
        elif self.plan == 1:
            if self.counter_ >= 7.853:
                self.counter_ = 0
                self.plan = 2
            else:
                self.posX = 0.4*math.cos(self.counter_)
                self.posY = 0
                self.counter_ += 0.04
                self.send_position_command()

        elif self.plan == 2:
            if self.counter_ >= 6.283:
                self.counter_ = 0
                self.plan = 0
            else:
                self.posX = 0.4*math.cos(self.counter_)
                self.posY = 0.4*math.sin(self.counter_)
                self.counter_ += 0.04
                self.send_position_command()


    def send_position_command(self):

        msg = SetPosition()

        Rx = np.array([[1, 0, 0],
                       [0, math.cos(self.posX), -math.sin(self.posX)],
                       [0, math.sin(self.posX), math.cos(self.posX)]])
        
        Ry = np.array([[math.cos(self.posY), 0, math.sin(self.posY)],
                       [0, 1, 0],
                       [-math.sin(self.posY), 0, math.cos(self.posY)]])
        
        R = np.matmul(Rx, Ry)

        BP_a = np.array([18.708, 0, 0])
        BP_b = np.array([-9.354, -16.202, 0])
        BP_c = np.array([-9.354, 16.202, 0])

        #ED_a = np.array([21.95, 0, 41])
        #ED_b = np.array([-10.975, -19.009, 41])
        #ED_c = np.array([-10.975, 19.009, 41])

        ED_a = np.array([11.27, 0, 41])
        ED_b = np.array([-5.64, -9.76, 41])
        ED_c = np.array([-5.64, 9.76, 41])

        R_a = np.matmul(R, ED_a)
        R_b = np.matmul(R, ED_b)
        R_c = np.matmul(R, ED_c)

        dA = math.sqrt(pow(R_a[0]-BP_a[0],2) + pow(R_a[1]-BP_a[1],2) + pow(R_a[2]-BP_a[2],2))
        dB = math.sqrt(pow(R_b[0]-BP_b[0],2) + pow(R_b[1]-BP_b[1],2) + pow(R_b[2]-BP_b[2],2))
        dC = math.sqrt(pow(R_c[0]-BP_c[0],2) + pow(R_c[1]-BP_c[1],2) + pow(R_c[2]-BP_c[2],2))

        A_rad = math.acos((pow(self.S_H_L,2) + pow(dA,2) - pow(self.LL,2))/(2*dA*self.S_H_L))
        B_rad = math.acos((pow(self.S_H_L,2) + pow(dB,2) - pow(self.LL,2))/(2*dB*self.S_H_L))
        C_rad = math.acos((pow(self.S_H_L,2) + pow(dC,2) - pow(self.LL,2))/(2*dC*self.S_H_L))
        
        self.get_logger().info("A Position" + str(A_rad))
        self.get_logger().info("B Position" + str(B_rad))
        self.get_logger().info("C Position" + str(C_rad))

        X = 212 + (A_rad - 0)*((512-212)/(1.5708-0))
        Y = 212 + (B_rad - 0)*((512-212)/(1.5708-0))
        Z = 212 + (C_rad - 0)*((512-212)/(1.5708-0))

        X = round(X)
        Y = round(Y)
        Z = round(Z)

        msg.id = 5
        msg.position = X
        self.cmd_pos_pub_.publish(msg)

        msg.id = 6
        msg.position = Y
        self.cmd_pos_pub_.publish(msg)

        msg.id = 7
        msg.position = Z
        self.cmd_pos_pub_.publish(msg)



        


def main(args=None):
    rclpy.init(args=args)
    node = xl320_Test_Node()
    rclpy.spin(node)
    rclpy.shutdown()
