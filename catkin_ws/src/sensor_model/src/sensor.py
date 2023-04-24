import numpy as np
import math
from sensor_msgs.msg import LaserScan
from motion_model.msg import motion_model_msgs
from nav_msgs.msg import OccupancyGrid
import rospy

class sensor:
    def __init__(self):
        rospy.init_node('sensor_model', anonymous=True)
        self.rate = rospy.Rate(10)

        # scan data
        self.all_sacns = []
        self.scan_max = 0

        # z intrisic parameters
        self.z_hit = 1
        self.z_random = 0
        self.z_max = 0

        # motion 
        self.x = 0
        self.y = 0
        self.yaw = 0

        # map
        reslf. d

        # distance matrix
        # coordinates are rounded to 2 floating points x.x
        self.dist_matrix = {}

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/motion_model', motion_model_msgs, self.motion_callback)

    
    def run(self):
        while True:
            self.rate.sleep()

    def scan_callback(self, msg):
        self.scan_max = msg.range_max
        self.all_scans = msg.ranges
        print(len(self.all_scans))
        print(self.scan_max)

    def motion_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw

    def likelihood_field_range_finder(self, ranges, x, y, yaw):
        q = 1
        for range in ranges:
            if range <= self.scan_max:      # suppose sensor is at the center of the robot
                endpoint_x = x + range * math.cos(yaw)
                endpoint_y = y + range * math.sin(yaw)

    def 


if __name__ == '__main__':
    sensor = sensor()
    sensor.run()
    
