#! /usr/bin/env python
import numpy as np
import rospy
import math
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from motion_model.msg import motion_model_msgs
import random

debug = False

class motion:
    def __init__(self):
        rospy.init_node("motion_model", anonymous=True)
        ### Actual ###
        # actual position updated by /Pose
        self.actual_x = 0
        self.actual_y = 0
        # actual orientation updated by /Pose
        self.actual_yaw = 0
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        ### Odometry ###
        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.pub = rospy.Publisher('/motion_model', motion_model_msgs, queue_size=10)
        
        self.rate = rospy.Rate(10)
        #rospy.spin()

    def run(self):
        #print(self.odom_x, self.odom_y, self.odom_yaw)
        #print(self.actual_x, self.actual_y, self.actual_yaw)    

        msg = motion_model_msgs()

        while not rospy.is_shutdown():
            cur_act_x = self.actual_x
            cur_act_y = self.actual_y
            cur_act_yaw = self.actual_yaw

            cur_act_x, cur_act_y, cur_act_yaw = self.sample_motion_model(
                self.odom_x, self.odom_y, self.odom_yaw,
                cur_act_x, cur_act_y, cur_act_yaw
            )
            msg.x = cur_act_x
            msg.y = cur_act_y
            msg.yaw = cur_act_yaw
            
            print(f"x: {msg.x}, y: {msg.y}, yaw: {round(cur_act_yaw, 4)}")

            self.pub.publish(msg)
            #self.rate.sleep()
            rospy.sleep(0.1)

    # update actual pose (x, y), and actual yaw 
    def pose_callback(self, msg):
        # read from gazebo/ModelStates
        index = msg.name.index("turtlebot3")
        pose = msg.pose[index]
        position = pose.position
        orientation = pose.orientation

        self.actual_x = float(position.x)
        self.actual_y = float(position.y)


        o_x = orientation.x
        o_y = orientation.y
        o_z = orientation.z
        o_w = orientation.w
        
        # caculate yaw
        self.actual_yaw = math.atan2(2 * (o_w * o_z + o_x * o_y),
                                     1 - 2 * (o_y ** 2 + o_z ** 2))
        
        if debug:
            print("--------------------")
            print(f"Actual \
                x: {round(self.actual_x, 3)}, \
                y: {round(self.actual_y, 3)}, \
                yaw: {round(self.actual_yaw, 3)}")
        
    
    def odom_callback(self, odom_msg):
        self.odom_x = odom_msg.pose.pose.position.x
        self.odom_y = odom_msg.pose.pose.position.y

        o_x = odom_msg.pose.pose.orientation.x
        o_y = odom_msg.pose.pose.orientation.y
        o_z = odom_msg.pose.pose.orientation.z
        o_w = odom_msg.pose.pose.orientation.w

        self.odom_yaw = math.atan2(2 * (o_w * o_z + o_x * o_y),
                                     1 - 2 * (o_y ** 2 + o_z ** 2))
        if debug: 
            print(f"Odometry \
                x: {round(self.odom_x, 3)}, \
                y: {round(self.odom_y, 3)}, \
                yaw: {round(self.odom_yaw, 3)}")
    
    def sample_motion_model(self, 
                            odom_x_p, odom_y_p, odom_yaw_p,
                            act_x_p, act_y_p, act_yaw_p):

        a1 = 0.1
        a2 = 0.1        
        a3 = 0.1
        a4 = 0.1

        # current odometry
        odom_x_c = self.odom_x
        odom_y_c = self.odom_y
        odom_yaw_c = self.odom_yaw
        
        delta_rot1 =  math.atan2(odom_y_c - odom_y_p, odom_x_c - odom_x_p) - odom_yaw_p
        delta_trans = math.sqrt(((odom_x_c - odom_x_p) ** 2) + ((odom_y_c - odom_y_p) ** 2))
        delta_rot2 = odom_yaw_c - odom_yaw_p - delta_rot1

        delta_rot1_cap = delta_rot1 - self.sample_normal_distribution(a1 * delta_rot1 + a2 * delta_trans)
        delta_trans_cap = delta_trans - self.sample_normal_distribution(a3 * delta_trans + a4 * (delta_rot1 + delta_rot2))
        delta_rot2_cap = delta_rot2 - self.sample_normal_distribution(a1 * delta_rot2 + a2 * delta_trans)

        x_next = act_x_p + delta_trans_cap * math.cos(act_yaw_p + delta_rot1_cap)
        y_next = act_y_p + delta_trans_cap * math.sin(act_yaw_p + delta_rot1_cap)
        yaw_next = act_yaw_p + delta_rot1_cap + delta_rot2_cap
        
        return x_next, y_next, yaw_next
        
    def sample_normal_distribution(self, x):
        total = 0
        for i in range(12):
            total += random.uniform(-x, x)
        return total / 2
        
if __name__ == '__main__':
    model = motion()
    model.run()


