#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64

gripper_position_pub = rospy.Publisher('/gripper/position_in_mm_sub', Float64, queue_size=10)

def gripper_position_command(msg):
    position = Float64()
    position = msg

    # limit
    if position <= 0:
        position = 0
    elif position >= 100:
        position = 100

    gripper_position_pub.publish(position)

def main():
    rospy.init_node("utra6_850_stepper_motor_node", anonymous=True)
    
    # init value
    

    while not rospy.is_shutdown():
        gripper_position = raw_input("Enter gripper position/ type 'e' to exit: ")
        if gripper_position == "e":
            sys.exit()

        gripper_position = float(gripper_position)
        gripper_position_command(gripper_position)
        print("position sent!")



if __name__ == "__main__":
    main() 