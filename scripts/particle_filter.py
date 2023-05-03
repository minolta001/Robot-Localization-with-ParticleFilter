#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Header, String
import random
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformListener
from tf import TransformBroadcaster
from numpy.random import random_sample
from copy import deepcopy

class sensor(object):
    def __init__(self):
        #rospy.init_node('sensor_model', anonymous=True)
        #self.rate = rospy.Rate(10)

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

        # map
        self.width = 0
        self.height = 0 
        rospy.wait_for_service("static_map") 
        house_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = house_map().map

        self.resolution = 0
        self.origin = 0

        self.dist_matrix = []
        #self.likelihood_matrix = []

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        #rospy.Subscriber('/motion_model', motion_model_msgs, self.motion_callback)

        self.run()

    
    def run(self):
        #grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
        self.width = self.map.info.width
        self.height = self.map.info.height
        self.resolution = self.map.info.resolution
        self.origin = self.map.info.origin

        #self.map = np.reshape(grid_msg.data, (self.height, self.width))
        #self.map = np.flipud(self.map)

        #self.dist_matrix = np.full((self.height, self.width), np.inf)
        #self.likelihood_matrix = np.zeros_like(self.map, dtype=np.float32)
        #self.likelihood_matrix = np.full((self.height, self.width), 1)

        
        dist_file_exist = True
        if dist_file_exist:
            self.dist_matrix = np.genfromtxt('dist_matrix.csv', delimiter=',') 
            print("Load dist matrix done: ", self.dist_matrix.shape)
        else:
            # dist_matrix initialize
            #self.dist_matrix = np.full((self.width * self.height, 3), -1)
            self.dist_matrix = np.full((self.width, self.height), -1.0)
            for i in range(self.width):
                for j in range(self.height):
                    idx = i + j * self.width            # !! important
                    if self.map.data[idx] > 0: 
                        self.dist_matrix[i, j] = 0 
            self.brushfire()

        '''
        #self.likelihood_field()
        count = 0
        while True:
            self.likelihood_field_range_finder(self.x, self.y, self.yaw)
            rospy.sleep(0.1)
            count += 1
            if count == 10:
                np.savetxt('likelihood_matrix.csv', self. likelihood_matrix, delimiter=',')
                count = 0
        '''
    


    def brushfire_propagate(self, x, y, val):
        val = val + self.resolution
        if(x >= 0 and x < self.width and y >= 0 and y < self.height):
            idx = x + y * self.width
            if(self.map.data[idx] < 100 and self.map.data[idx] >= 0):
                    cur = self.dist_matrix[x, y]
                    if(cur == -1):
                        print(round(val, 3))
                        self.dist_matrix[x, y] = round(val, 3)
                        print(self.dist_matrix[x, y])
                        return
                    if(val < cur):
                        self.dist_matrix[x, y] = round(val, 3)
                        return



    def brushfire(self):
        # propagating
        for k in range(70):         # max_range / map.resolution
            print(f"Brushfire {k}")
            for i in range(self.width):
                for j in range(self.height): 
                    #idx = i + j * self.width
                    if(self.dist_matrix[i, j] >= 0):
                        val = self.dist_matrix[i, j]
                        self.brushfire_propagate(i - 1, j - 1, val)
                        self.brushfire_propagate(i, j - 1, val)
                        self.brushfire_propagate(i + 1, j - 1, val)
                        self.brushfire_propagate(i - 1, j, val)
                        self.brushfire_propagate(i + 1, j, val)
                        self.brushfire_propagate(i - 1, j + 1, val)
                        self.brushfire_propagate(i, j + 1, val)
                        self.brushfire_propagate(i + 1, j + 1, val)

            print(np.nanmin(self.dist_matrix))
            print(np.nanmax(self.dist_matrix))
    
        #np.savetxt('dist_matrix.txt', self.dist_matrix, fmt='%.2f')
        np.savetxt('dist_matrix.csv', self.dist_matrix, delimiter=',')

    


    def scan_callback(self, scan_msg):
        self.scan_max = scan_msg.range_max
        self.all_scans = scan_msg.ranges
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment






def get_yaw(p):
    yaw = euler_from_quaternion([
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w])[2]
    return yaw

def sample_normal_distribution(x):
        total = 0
        for i in range(12):
            total += random.uniform(-x, x)
        return total / 2

def pdf_gaussian(x, cov):
    return 1 / (math.sqrt(2 * math.pi) * cov) * math.exp(-(x ** 2) / (2 * cov ** 2))


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """ 
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples

    



class particle:
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight
    
    def particle_info(self):
        theta = euler_from_quaternion([
            self.pose.orientation.x, 
            self.pose.orientation.y, 
            self.pose.orientation.z, 
            self.pose.orientation.w])[2]
        return ("Particle: [" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(theta) + "]")


class ParticleFilter(object):
    def __init__(self):

        self.initialized = False

        self.base_frame = 'base_footprint'

        rospy.init_node('turtlebot3_particle_filter')
        
        self.particles_pub = rospy.Publisher('particle_cloud', PoseArray, queue_size=10)

        self.robot_estimate_pub = rospy.Publisher('estimated_robot_pose', PoseStamped, queue_size=10)

        self.map = OccupancyGrid()
        self.sensor_model = sensor()
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
    
        
        self.particles_num = 500

        # all scans data
        self.all_scans = []
        self.angle_min = 0
        self.angle_increment = 0

        # pose
        self.robot_pose_estimate = Pose()
        self.prev_odom_pose = None


        self.particles_initialize()

        rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        rospy.Subscriber('scan', LaserScan, self.range_finder)

        self.initialized = True 

    # initialize particles list
    def particles_initialize(self):
        self.particles_list = []
        for i in range(self.particles_num):
            x = random.uniform(-8, 8)
            y = random.uniform(-8, 8)
            theta = random.uniform(0, 359 / 180 * math.pi)
            
            p = Pose()
            p.position = Point()
            p.position.x = x
            p.position.y = y
            p.position.z = 0
            p.orientation = Quaternion()

            q = quaternion_from_euler(0.0, 0.0, theta)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            
            par = particle(p, 1.0)
            self.particles_list.append(par) 


        self.normalize_particles()
        self.particles_publish()




    # Get delta values from robot odometry, update particles pose with noise
    def particles_motion_model(self):   # update particles based on motion_model
        a1 = 0.1
        a2 = 0.1
        a3 = 0.1
        a4 = 0.1 

        cur_x = self.odom_pose.pose.position.x
        cur_y = self.odom_pose.pose.position.y
        cur_yaw = get_yaw(self.odom_pose.pose)

        prev_x = self.prev_odom_pose.pose.position.x
        prev_y = self.prev_odom_pose.pose.position.y
        prev_yaw = get_yaw(self.prev_odom_pose.pose)

        delta_rot1 = math.atan2(cur_y - prev_y, cur_x - prev_x) - prev_yaw
        delta_trans = math.sqrt(((cur_x - prev_x) ** 2) + ((cur_y - prev_y) ** 2))
        delta_rot2 = cur_yaw - prev_yaw - delta_rot1

        # update particle next pose, with given delta cap
        '''
        for par in self.particles_list:
            delta_rot1_cap = delta_rot1 - sample_normal_distribution(a1 * delta_rot1 + a2 * delta_trans)
            delta_trans_cap = delta_trans - sample_normal_distribution(a3 * delta_trans + a4 * (delta_rot1 + delta_rot2))
            delta_rot2_cap = delta_rot2 - sample_normal_distribution(a1 * delta_rot2 + a2 * delta_trans)

            new_pose = Pose()
            new_pose.position.x = par.pose.position.x + delta_trans_cap * math.cos(get_yaw(par.pose) + delta_rot1_cap)
            new_pose.position.y = par.pose.position.y + delta_trans_cap * math.sin(get_yaw(par.pose) + delta_rot1_cap)
            new_yaw = get_yaw(par.pose) + delta_rot1_cap + delta_rot2_cap 

            qua = quaternion_from_euler(0.0, 0.0, new_yaw)
            new_pose.orientation.x = qua[0]
            new_pose.orientation.y = qua[1]
            new_pose.orientation.z = qua[2]
            new_pose.orientation.w = qua[3]
            
            par.pose = new_pose
        '''

        for par in self.particles_list:
            px = par.pose.position.x
            py = par.pose.position.y
            pyaw = get_yaw(par.pose)

            delta_rot1_cap = delta_rot1 - sample_normal_distribution(a1 * delta_rot1 + a2 * delta_trans)
            delta_trans_cap = delta_trans - sample_normal_distribution(a3 * delta_trans + a4 * (delta_rot1 + delta_rot2))
            delta_rot2_cap = delta_rot2 - sample_normal_distribution(a1 * delta_rot2 + a2 * delta_trans)


            next_x = px + delta_trans_cap * math.cos(pyaw + delta_rot1_cap)
            next_y = py + delta_trans_cap * math.sin(pyaw + delta_rot1_cap)
            #x_coor = int((next_x - self.sensor_model.origin.position.x) / self.sensor_model.resolution)
            #y_coor = int((next_y - self.sensor_model.origin.position.y) / self.sensor_model.resolution)

            par.pose.position.x = next_x
            par.pose.position.y = next_y
            yaw_next = pyaw + delta_rot1_cap + delta_rot2_cap
            q = quaternion_from_euler(0.0, 0.0, yaw_next)
            par.pose.orientation.x = q[0]
            par.pose.orientation.y = q[1]
            par.pose.orientation.z = q[2]
            par.pose.orientation.w = q[3]

    
    def particles_sensor_model(self):
        #scan_angle = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330]
        scan_angle = [0, 45, 90, 135, 180, 225, 270, 315]
        for par in self.particles_list:
            #x_coor = int((x - self.sensor_model.origin.position.x) / self.sensor_model.resolution)
            #y_coor = int((y - self.sensor_model.origin.position.y) / self.sensor_model.resolution)

            x = par.pose.position.x
            y = par.pose.position.y
            yaw = get_yaw(par.pose)
           
            #TODO: Log of P? 


            q = 1
            
            for angle in scan_angle:
                scan_range = self.all_scans[angle]

                if scan_range <= 3.5:
                    endpoint_x = int(x + scan_range * math.cos(yaw + (self.angle_min + angle * self.angle_increment)))
                    endpoint_y = int(y + scan_range * math.sin(yaw + (self.angle_min + angle * self.angle_increment)))

                    x_coor = int((endpoint_x - self.sensor_model.origin.position.x) / self.sensor_model.resolution)
                    y_coor = int((endpoint_y - self.sensor_model.origin.position.y) / self.sensor_model.resolution)

                     
                    #print(f"end x: {endpoint_x}, end y: {endpoint_y}")
                    dist = 0
                    if(x_coor >= 384 or y_coor >= 384):
                        dist = -1
                    else:
                        dist = self.sensor_model.dist_matrix[x_coor, y_coor]


                    if dist != -1: 
                        prob = pdf_gaussian(dist, 10)
                        q = q * prob
                    else:
                        q = q * 0.000001
            if q != 0:
                par.weight = q
            else:
                print("enter")
                par.weight = 0.000001

            
        

    def normalize_particles(self):
        '''
        weight_sum = 0
        for p in self.particles_list:
            weight_sum = weight_sum + p.weight
        '''

        weight_sum = sum([par.weight for par in self.particles_list])

        for p in self.particles_list:
            p.weight = p.weight / weight_sum


    def resample_particles(self):
        '''
        probs = []  # probabilities (weight)
        for par in self.particles_list:
            probs.append(par.weight)
        '''

        particles_weights = [particle.weight for particle in self.particles_list]

        new_particles_list = draw_random_sample(self.particles_list, particles_weights, self.particles_num)
        self.particles_list = new_particles_list


        '''
        self.particles_list = []
        for par in new_particles_list:
            pose = par.pose
            weight = par.weight
            new_p = particle(pose, weight)
            self.particles_list.append(new_p)
        '''

        '''
        beta = 0.0
        self.normalize_particles()
        new_list = []
        test_list = []
        l = len(self.particles_list)
        index = int(random.random() * l)
        max_prob = np.max(probs)

        for i in range(l):
            beta += random.random() * 2.0 * max_prob
            while beta > probs[index]:
                beta -= probs[index]
                index = (index + 1) % l

            test_list.append(probs[index])
            new_list.append(self.particles_list[index])

        #print(test_list) 
        self.particles_list = new_list
        '''

    def update_robot_estimate_pose(self):
        x_sum = 0
        y_sum = 0
        ox_sum = 0
        oy_sum = 0
        oz_sum = 0
        ow_sum = 0
        
        for par in self.particles_list:
            weight = par.weight
            x = par.pose.position.x * weight
            y = par.pose.position.y * weight
            ox = par.pose.orientation.x * weight
            oy = par.pose.orientation.y * weight
            oz = par.pose.orientation.z * weight
            ow = par.pose.orientation.w * weight
            
            x_sum += x 
            y_sum += y
            ox_sum += ox
            oy_sum += oy
            oz_sum += oz
            ow_sum += ow
        
        self.robot_pose_estimate.position.x = x_sum
        self.robot_pose_estimate.position.y = y_sum
        self.robot_pose_estimate.orientation.x = ox_sum
        self.robot_pose_estimate.orientation.y = oy_sum
        self.robot_pose_estimate.orientation.z = oz_sum
        self.robot_pose_estimate.orientation.w = ow_sum
    

        
    
    def map_callback(self, map_data):
        self.map = map_data
    
    def particles_publish(self):
        particles_pose_array = PoseArray()
        particles_pose_array.header = Header(stamp=rospy.Time.now(), frame_id='map')
        particles_pose_array.poses

        for par in self.particles_list:
            particles_pose_array.poses.append(par.pose)

        self.particles_pub.publish(particles_pose_array)


    def robot_estimate_pose_publish(self):
        robot_pose_stamped = PoseStamped()
        robot_pose_stamped.pose = self.robot_pose_estimate
        robot_pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.robot_estimate_pub.publish(robot_pose_stamped)

    
    def range_finder(self, scan_data):
        self.angle_min = scan_data.angle_min
        self.angle_increment = scan_data.angle_increment
        self.all_scans = scan_data.ranges

        
       
        if not self.initialized:
            return
    
        if not(self.tf_listener.canTransform(self.base_frame, scan_data.header.frame_id, scan_data.header.stamp)):
            return
        
        self.tf_listener.waitForTransform(self.base_frame, "odom", scan_data.header.stamp, rospy.Duration(2)) 

        if not(self.tf_listener.canTransform(self.base_frame, scan_data.header.frame_id, scan_data.header.stamp)): 
            return
        
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=scan_data.header.frame_id))

        # transform scan frame to base_footprint frame 
        self.scan_pose = self.tf_listener.transformPose(self.base_frame, p)
        
        p = PoseStamped(header=Header(stamp=scan_data.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        
        # transform base_frame to odom_frame (robot pose based on odometry)
        self.odom_pose = self.tf_listener.transformPose("odom", p)

        #i = round(self.odom_pose.pose.position.x, 2)
        #j = round(self.odom_pose.pose.position.y, 2)
        #x_coor = int((i - self.sensor_model.origin.position.x) / self.sensor_model.resolution)
        #y_coor = int((j - self.sensor_model.origin.position.y) / self.sensor_model.resolution)
        #print(x_coor, y_coor)
        #print(self.sensor_model.dist_matrix[x_coor, y_coor])
        #print("--------------------")
    
        if not self.prev_odom_pose:
            self.prev_odom_pose = self.odom_pose
            return
        
        #if len(self.particles_list) != 0:
        if self.particles_list:
            cur_x = self.odom_pose.pose.position.x
            cur_y = self.odom_pose.pose.position.y
            cur_yaw = get_yaw(self.odom_pose.pose)

            prev_x = self.prev_odom_pose.pose.position.x
            prev_y = self.prev_odom_pose.pose.position.y
            prev_yaw = get_yaw(self.prev_odom_pose.pose)

            if(np.abs(cur_x - prev_x) > 0.1 or
               np.abs(cur_y - prev_y) > 0.1 or
               np.abs(cur_yaw - prev_yaw) > math.pi / 6):
                self.particles_motion_model()

                self.particles_sensor_model()

                self.normalize_particles()
            
                self.resample_particles()

                self.update_robot_estimate_pose()

                self.particles_publish()

                self.robot_estimate_pose_publish()
                
                self.prev_odom_pose = self.odom_pose
            


    

        
        

if __name__ == '__main__':
    #sensor = sensor()
    #sensor.run()
    
    pf = ParticleFilter()
    rospy.spin()