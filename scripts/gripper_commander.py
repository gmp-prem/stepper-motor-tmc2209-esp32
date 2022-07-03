#!/usr/bin/env python3

import rospy

# msg
from std_msgs.msg import Float64, UInt8

# srv
from std_srvs.srv import Empty

# common
from scipy.interpolate import InterpolatedUnivariateSpline

class GripperCommander():
    def __init__(self, test=False):
        print("gripper commander node init")

        self.test = test

        # publishers
        self.position_in_mm_pub = rospy.Publisher("gripper/position_in_mm_sub", Float64, queue_size=5)
        self.speed_in_mm_per_sec_pub = rospy.Publisher("gripper/speed_inmm_sub", Float64, queue_size=5)
        self.speed_in_step_per_sec_pub = rospy.Publisher("gripper/speed_sub", Float64, queue_size=5)

        # subscribers
        rospy.Subscriber("gripper/position_goal", Float64, self.gripper_position_callback)
        rospy.Subscriber("gripper/state", UInt8, self.gripper_state_callback)

        self.gripper_range = [0, 141]
        self.gripper_open = float(self.gripper_range[1])
        self.gripper_close = float(self.gripper_range[0])
        self.gripper_standby = float(75)

        x = [-11,5,10,30,78,110,134,141] # actual value measure
        y = [100,93,90,80,60,40,20,0] # value to send to stepper motor

        self.mm2step_gripper = InterpolatedUnivariateSpline(x, y, k=5, check_finite=False)

        print(f"gripper range {self.gripper_range[0]} to {self.gripper_range[1]}")
        print("gripper commander is ready to take commmand")

    def position_in_mm_control(self, x):
        position_in_mm_msg = Float64()

        position_in_mm_msg.data = self.mm2step_gripper(x)
        print(f"step in mm actual : {self.mm2step_gripper(x)}")
        
        self.position_in_mm_pub.publish(position_in_mm_msg)

    def speed_in_mm_per_sec_control(self, x):
        speed_in_mm_per_sec_msg = Float64()
        speed_in_mm_per_sec_msg.data = x
        self.speed_in_mm_per_sec_pub.publish(speed_in_mm_per_sec_msg)

    def speed_in_step_per_sec_control(self, x):
        speed_in_step_per_sec_msg = Float64()
        speed_in_step_per_sec_msg.data = x
        self.speed_in_step_per_sec_pub.publish(speed_in_step_per_sec_msg)

    def gripper_state_callback(self, msg):
        self.gripper_state = msg.data

    def gripper_position_callback(self, msg):
        self.gripper_goal_position = msg.data

    def intit_home(self,):
        print("init gripper home")

        rospy.wait_for_service('/gripper/init_home')
        
        home_stepper = rospy.ServiceProxy('/gripper/init_home', Empty)
        resp1 = home_stepper()

        print('gripper homing finished')

    def close(self, ):
        self.position_in_mm_control(self.gripper_close)

    def open(self, ):
        self.position_in_mm_control(self.gripper_open)

    def standby(self, ):
        self.position_in_mm_control(self.gripper_standby)    

    def process(self, ):
        if self.test:
            self.intit_home()

        while not rospy.is_shutdown():
            if self.test:
                x = input()
                self.position_in_mm_control(float(x))

            rospy.spin()

    
if __name__ == "__main__":
    try:
        rospy.init_node("gripper_commander_node", anonymous=True)
        gripper_commander = GripperCommander(test=True)
        gripper_commander.process()
    except rospy.ROSInterruptException:
        pass

