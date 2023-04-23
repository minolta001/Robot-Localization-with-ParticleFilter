import numpy as np
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import random

debug = True

class motion_model:
    def __init__(self):
        rospy.init_node("motion_model", anonymous=True)

        ### Actual ###
        # actual position updated by /Pose
        self.actual_x = 0
        self.actual_y = 0
        # actual orientation updated by /Pose
        self.actual_yaw = 0
        rospy.Subscriber('/pose', PoseStamped, self.pose_callback)

        ### Odometry ###
        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def run(self):
        cur_act_x = self.actual_x
        cur_act_y = self.actual_y
        cur_act_yaw = self.actual_yaw
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            cur_act_x, cur_act_y, cur_act_yaw = self.sample_motion_model(
                self.odom_x, self.odom_y, self.odom_yaw,
                cur_act_x, cur_act_y, cur_act_yaw
            )
            
            rate.sleep()

    # update actual pose (x, y), and actual yaw 
    def pose_callback(self, pose_msg):
        self.actual_x = float(pose_msg.pose.position.x)
        self.actual_y = float(pose_msg.pose.position.y)

        o_x = pose_msg.pose.orientation.x
        o_y = pose_msg.pose.orientation.y
        o_z = pose_msg.pose.orientation.z
        o_w = pose_msg.pose.orientation.w
        
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
        delta_rot2 = odom_yaw_c - odom_y_p - delta_rot1
    
        delta_rot1_cap = delta_rot1 - self.sample_normal_distribution(a1 * (delta_rot1 ** 2) + a2 * (delta_trans ** 2))
        delta_trans_cap = delta_trans - self.sample_normal_distribution(a3 * (delta_trans ** 2) + a4 * ((delta_rot1 ** 2) + (delta_rot2 ** 2)))
        delta_rot2_cap = delta_rot2 - self.sample_normal_distribution(a1 * (delta_rot2 ** 2) + a2 * (delta_trans ** 2))

        x_next = act_x_p + delta_trans_cap * math.cos(act_yaw_p + delta_rot1_cap)
        y_next = act_y_p + delta_trans_cap * math.sin(act_yaw_p + delta_rot1_cap)
        yaw_next = act_yaw_p + delta_rot1_cap + delta_rot2_cap

        return x_next, y_next, yaw_next
        
    def sample_normal_distribution(self, x):
        total = 0
        for i in range(12):
            total += random.uniform(-x, x)
        return total / 2
        
        


