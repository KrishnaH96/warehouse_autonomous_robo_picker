#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard
from sympy import *
import sympy as sp
import matplotlib.pyplot as plt
import numpy as np
from sympy.utilities.lambdify import lambdify
from scipy.optimize import minimize

import time
# Define key codes
LIN_VEL_STEP_SIZE = 0.1


class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def run_keyboard_control(self):
        # self.msg = """
        # Control Your Car!
        # ---------------------------
        # Moving around:
        #     w
        # a    s    d

        # q : force stop

        # Esc to quit
        # """

        #self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        LIN_VEL_STEP_SIZE=0.05
        joint1=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    joint1 = 0.0

                elif key == 'w':  # Forward
                    joint1 +=LIN_VEL_STEP_SIZE
                    print(joint1)
                elif key == 's':  # Reverse
                    joint1 -= LIN_VEL_STEP_SIZE
                    print(joint1)
                joint_positions.data = [joint1]
                msg = Float64MultiArray()
                msg.data = [0.2]

                self.joint_position_pub.publish(msg)
                


                velocities = np.linspace(0.0, -0.5, num=50)
                delay_time = 0.1  # Delay time in seconds (100 ms)

                for velocity in velocities:
                    # Publish the current velocity
                    msg = Float64MultiArray()
                    msg.data = [velocity]

                    self.joint_position_pub.publish(msg)
                    
                    # Add delay between publications
                    time.sleep(delay_time)
                time.sleep(1)

                velocities = np.linspace(-0.5, 0.2, num=50)
                delay_time = 0.1  # Delay time in seconds (100 ms)

                for velocity in velocities:
                    # Publish the current velocity
                    msg = Float64MultiArray()
                    msg.data = [velocity]

                    self.joint_position_pub.publish(msg)
                    
                    # Add delay between publications
                    time.sleep(delay_time)

                time.sleep(10)


                #self.joint_position_pub.publish(joint_positions)



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
