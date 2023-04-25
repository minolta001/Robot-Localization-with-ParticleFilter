import numpy as np
import math
from sensor_msgs.msg import LaserScan
from motion_model.msg import motion_model_msgs
from nav_msgs.msg import OccupancyGrid
import rospy
import random

class sensor:
    def __init__(self):
        rospy.init_node('sensor_model', anonymous=True)
        self.rate = rospy.Rate(10)

        # scan data
        self.all_sacns = []
        self.scan_max = 0

        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0

        # z intrisic parameters
        self.z_hit = 1
        self.z_random = 0
        self.z_max = 0

        # motion 
        self.x = 0
        self.y = 0
        self.yaw = 0

        # map
        self.width = 0
        self.height = 0
        self.map = []
        self.resolution = 0

        # distance matrix
        # coordinates are rounded to 2 floatintableg points x.x
        self.likelihood_matrix = []

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/motion_model', motion_model_msgs, self.motion_callback)

    
    def run(self):
        grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
        self.width = grid_msg.info.width
        self.height = grid_msg.info.height
        self.resolution = grid_msg.info.resolution
        self.map = np.reshape(grid_msg.data, (self.height, self.width))
        self.map = np.flipud(self.map)

        self.dist_matrix = np.full((self.height, self.width), np.inf)
        self.likelihood_matrix = np.zeros_like(self.map, dtype=np.float32)
        #self.brushfire()
        self.

    

    def brushfire_propagate(self, x, y, val):
        if(x >= 0 and x < self.map.shape[0] and y >= 0 and y < self.map.shape[1]):
            if(self.dist_matrix[x, y] != -1):
                if(self.dist_matrix[x, y] != math.inf):
                    val = val + self.resolution
                    cur = self.dist_matrix[x, y]
                    if val < cur:
                        self.dist_matrix[x, y] = val

                elif(self.dist_matrix[x, y] == math.inf):
                    self.dist_matrix[x, y] = val + self.resolution
                
                #if self.dist_matrix[x, y] > 0.2:
                    #self.dist_matrix[x, y] = 10



    def brushfire(self):
        # initialize dist_matrix
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if(self.map[i, j] > 90):
                    self.dist_matrix[i, j] = 0
                elif(self.map[i, j] == -1):
                    self.dist_matrix[i, j] = -1

        # propagating
        for k in range(200):         # max_range / map.resolution
            print(f"Brushfire {k}")
            for i in range(self.map.shape[0]):
                for j in range(self.map.shape[1]):
                    if(self.dist_matrix[i, j] != math.inf and self.dist_matrix[i, j] != -1):
                        val = self.dist_matrix[i, j]
                        self.brushfire_propagate(i - 1, j - 1, val)
                        self.brushfire_propagate(i, j - 1, val)
                        self.brushfire_propagate(i + 1, j - 1, val)
                        self.brushfire_propagate(i - 1, j, val)
                        self.brushfire_propagate(i + 1, j, val)
                        self.brushfire_propagate(i - 1, j + 1, val)
                        self.brushfire_propagate(i, j + 1, val)
                        self.brushfire_propagate(i + 1, j + 1, val)


        max_value = np.max(self.dist_matrix)
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if(self.dist_matrix[i, j] > 0.2):
                    self.dist_matrix[i, j] = max_value

        #np.savetxt('dist_matrix.txt', self.dist_matrix, fmt='%.2f')
        np.savetxt('dist_matrix.csv', self.dist_matrix, delimiter=',')


    def scan_callback(self, scan_msg):
        self.scan_max = scan_msg.range_max
        self.all_scans = scan_msg.ranges
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        


    def motion_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw

    # find the coordinates that is occupied and cloest to 
    def find_dist_min_pair(self, x, y):
        min_val = float('inf')
        dist_sq = 0
        for i in range(self.map.shape[0]):
            for j in range (self.map.shape[1]):
                if(self.map[i, j] >= 80):
                    dist_sq = (x - i) ** 2 + (y - j) ** 2
                    if dist_sq < min_val:
                        min_val = dist_sq
        return min_val

    # calculate probability of x under zero-centered gaussian model with covariance cov
    def pdf_gaussian(self, x, cov):
        return 1 / (math.sqrt(2 * math.pi * cov)) * math.exp(-(x ** 2) / (2 * cov))
    

    # compute distance
    def distance(self, rx, ry, end_x, end_y):
        return ((rx - end_x) * self.resolution) ** 2 + ((ry - end_y) * self.resolution) ** 2


    def all_grid_likelihood_field_range_finder(self):
        num_beams = 360

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                print(i, j)
                if(self.map[i, j] < 90 and self.map[i,j] != -1):
                    q = 1
                    for beam in num_beams:
                        angle = self.angle_min + beam * self.angle_increment
                        end_x = i       # set endpoint x at i first
                        end_y = j       # set endpoint y at j first

                        while end_x >= 0 and end_x < self.width and end_y >= 0 and end_y < self.height:
                            end_x = end_x + math.cos(angle)
                            end_y = end_y + math.sin(angle)
                            
                            if(self.map[end_x, end_y] >= 90 or self.map[end_x, end_y] == -1):
                                break

                        x = i
                        y = j

                        # this is the range in (for range in ranges)
                        range_dist = self.distance(x, y, end_x, end_y)      # range_dist is in 0.5 meter scale

                        if range_dist < self.scan_max:
                            dist_sq = self.find_dist_min_pair(x, y, end_x, end_y)
                            q = q * self.pdf_gaussian(dist_sq, 0.1)
                    self.likelihood_matrix[i, j] = round(q, 4)
        
        #np.savetxt('likelihood_matrix.txt', self.dist_matrix, fmt='%.2f')
        np.savetxt('likelihood_matrix.csv', self.likelihood_matrix, delimiter=',')

 



'''
    def likelihood_field_range_finder(self, x, y, yaw):
        q = 1
        for range in ranges:
            if range <= self.scan_max:      # suppose sensor is at the center of the robot
                endpoint_x = round(x + range * math.cos(yaw), 4)   # round to 4 floating point
                endpoint_y = round(y + range * math.sin(yaw), 4)
                dist_sq, min_x, min_y = self.find_dist_min_pair(endpoint_x, endpoint_y)
                q = q * self.pdf_gaussian(dist_sq, 0.1)
'''



if __name__ == '__main__':
    sensor = sensor()
    sensor.run()
    