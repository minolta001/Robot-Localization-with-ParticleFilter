#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from likelihood_field import LikelihoodField
import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random


def probability(dist, sigma_hit):

    # Calculate probability of Gaussian distribution
    c = 1.0 / (sigma_hit * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sigma_hit, 2)))
    return prob

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


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

def low_variance_sample(particles, weights, num_particles):
    new_particles = []
    M = num_particles
    r = np.random.uniform(0, 1/M)
    c = weights[0]
    i = 0
    for m in range(M):
        U = r + m/M
        while U > c:
            i += 1
            c = c+ weights[i]
        new_particles.append(deepcopy(particles[i]))
    return new_particles

class SampleMotionModel:
    def __init__(self):
        # Initialize the noise
        self.alpha = [0.1,0.1,0.1,0.1]
        # Initialize last Odometry
        self.prev_odometry = None

    def get_pose(self, odometry):
        x = odometry.pose.position.x
        y = odometry.pose.position.y
        quaternion = (odometry.pose.orientation.x, 
                odometry.pose.orientation.y, 
                odometry.pose.orientation.z,
                odometry.pose.orientation.w)
        _,_, theta = euler_from_quaternion(quaternion)
        #print(x,y,theta)

        return x, y, theta

    def calculate_pose_control(self, prev_odometry, cur_odometry):
        # Use get_pose function to get previous pose and current pose
        x1, y1, theta1 = self.get_pose(prev_odometry)
        #print(x1,y1,theta1)
        x2, y2, theta2 = self.get_pose(cur_odometry)
        #print(x2,y2,theta2)
        #print((x2-x1), (y2-y1), (theta2-theta1))
        #print(round((x2-x1),10), round((y2-y1),10), round((theta2-theta1),10))
        delta_trans = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        delta_rot1 = math.atan2(y2-y1, x2-x1)-theta1
        delta_rot2 = theta2-theta1-delta_rot1
        #print(theta1)
        #print(round(delta_rot1,4), round(delta_rot2,4))
        return delta_trans, delta_rot1, delta_rot2


    def sample_normal_distribution(self, b):
        rand_sum = sum(np.random.uniform(-b,b) for _ in range(12))
        return 0.5 * rand_sum

    def sample_motion_model(self, delta_trans, delta_rot1, delta_rot2, alpha, pose):

        delta_rot1_hat = delta_rot1 + self.sample_normal_distribution(alpha[0]*abs(delta_rot1) + alpha[1]*delta_trans)
        delta_trans_hat = delta_trans + self.sample_normal_distribution(alpha[2]*delta_trans + alpha[3]*(abs(delta_rot1) + abs(delta_rot2)))
        delta_rot2_hat = delta_rot2 + self.sample_normal_distribution(alpha[0]*abs(delta_rot2) + alpha[1]*delta_trans)

        new_pose = Pose()
        new_pose.position.x = pose.position.x + delta_trans_hat * math.cos(get_yaw_from_pose(pose) + delta_rot1_hat)
        new_pose.position.y = pose.position.y + delta_trans_hat * math.sin(get_yaw_from_pose(pose) + delta_rot1_hat)
        new_yaw = get_yaw_from_pose(pose) + delta_rot1_hat + delta_rot2_hat

        quaternion = quaternion_from_euler(0, 0, new_yaw)
        new_pose.orientation.x = quaternion[0]
        new_pose.orientation.y = quaternion[1]
        new_pose.orientation.z = quaternion[2]
        new_pose.orientation.w = quaternion[3]

        return new_pose
        
class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    '''def __repr__(self):
        return f"Particle(pose={self.pose}, weight={self.w})"'''

class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        
        self.distance_matrix = []

        self.dist_file_exist = True
        if self.dist_file_exist:
            self.distance_matrix = np.genfromtxt('dist_matrix.csv', delimiter= ',')
        # keep track whether the particle clouds are updated
        #self.particle_updated = False

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')
        # self.rate = rospy.Rate(10)
        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the estimated robot pose
        self.robot_estimate = Pose()
        # initialize the particle cloud array 
        self.prev_odometry = None

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        self.motion_model = SampleMotionModel()
        self.likelihood_field = LikelihoodField()
        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        # Get map info
        # print('Received map data')
        self.map = data
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_origin = data.info.origin
        self.map_resolution = data.info.resolution
        self.map_data = np.reshape(data.data, (self.map_width, self.map_height))
        self.map_receive = True

        
    
    def initialize_particle_cloud(self):
        # Initialize the particle sets
        self.particle_cloud = []

        x_ranges = (-8, 8)
        y_ranges = (-8, 8)
        theta_ranges = (-np.pi, np.pi)

        for i in range(self.num_particles):
            pose = Pose()
            pose.position = Point()
            pose.position.x = np.random.uniform(x_ranges[0], x_ranges[1])
            pose.position.y = np.random.uniform(y_ranges[0], y_ranges[1])
            pose.position.z = 0

            pose.orientation = Quaternion()
            quaternion = quaternion_from_euler(0.0, 0.0, np.random.uniform(theta_ranges[0], theta_ranges[1]))
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            # Create a new_particle with equal weight
            new_particle = Particle(pose, 1.0)
            self.particle_cloud.append(new_particle)

        print('Step1: Initializing particle clouds')
        self.normalize_particles()
        # print(self.particle_cloud)
        # Show the memory address
        self.publish_particle_cloud()
        # print("Initialized", len(self.particle_cloud), "particles")



    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        total_weight = sum([particle.w for particle in self.particle_cloud])
        for particle in self.particle_cloud:
            particle.w /= total_weight
        print('Step2: Normalizing particles')



    def publish_particle_cloud(self):
        print('Step3: Publishing particle clouds')
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)
        self.particles_pub.publish(particle_cloud_pose_array)



    def publish_estimated_robot_pose(self):
        print('Publishing esitimated robot pose')
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        print('Resampling particles')
        # Resample particles based on the weights
        particle_weights = [particle.w for particle in self.particle_cloud]

        #Use draw_random_sample function
        resample_particles = draw_random_sample(self.particle_cloud, particle_weights, self.num_particles)
        self.particle_cloud = resample_particles



    def robot_scan_received(self, data):
        # wait until initialization is complete
        if not(self.initialized):
            print('not initialized')
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print('not able to transform the laser frame to the base frame')
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            print('footprint not updated')
            return
        # print('robot scan received')
        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        #robot_dist = self.likelihood_field.get_closest_obstacle_distance(self.odom_pose.pose.position.x, self.odom_pose.pose.position.y)
        #print('********************Robot dist is: ',robot_dist,)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:
            #print(self.particle_cloud)

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out
                self.update_particles_with_motion_model()

                # self.save_file(self.occupied_cells(), 'occupied_cells.txt')

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()

                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose

                #self.particle_updated = True



    def update_estimated_robot_pose(self):
        print('update estimated robot pose is being called')
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # Initialize the robot_estimate pose
        robot_estimate_x = 0
        robot_estimate_y = 0
        robot_estimate_theta = 0

        # Calculate the total weight for all particles
        total_weight = sum([particle.w for particle in self.particle_cloud])
        for particle in self.particle_cloud:
            robot_estimate_x += particle.pose.position.x * particle.w / total_weight
            robot_estimate_y += particle.pose.position.y * particle.w / total_weight
            robot_estimate_theta += get_yaw_from_pose(particle.pose) * particle.w / total_weight

        # Normalize the theta angle
        while robot_estimate_theta > math.pi:
            robot_estimate_theta -= 2 * math.pi
        while robot_estimate_theta < math.pi:
            robot_estimate_theta += 2 * math.pi

        # Update the estimate position and orientation
        self.robot_estimate.position.x = robot_estimate_x
        self.robot_estimate.position.y = robot_estimate_y
        self.robot_estimate.position.z = 0
        quaternion = quaternion_from_euler(0, 0, robot_estimate_theta)
        self.robot_estimate.orientation.x = quaternion[0]
        self.robot_estimate.orientation.y = quaternion[1]
        self.robot_estimate.orientation.z = quaternion[2]
        self.robot_estimate.orientation.w = quaternion[3]
        # print(self.robot_estimate)



    def occupied_cells(self):
        print('Step 4.5: Getting occupied cells')
        occupied_list = []
        print(self.map_height)
        print(self.map_width)

        for i in range(self.map_width):
            for j in range(self.map_height):
                if self.map_data[i,j] > 0:
                    x = self.map_origin.position.x + i * self.map_resolution
                    y = self.map_origin.position.y + j * self.map_resolution
                    occupied_list.append((x,y))
        return occupied_list
    

    
    def save_file(self, occupied_list, filename = 'occupied_cells.txt'):
        with open(filename, 'w') as f:
            for cell in occupied_list:
                f.write(f'{cell[0]} {cell[1]}\n')
        print('Save file1')



    def update_particle_weights_with_measurement_model(self, data):
        print('Step5: Updating particle weights with measurement model')
        z_t = [0, 45, 90, 135, 180, 225, 270, 310, 355]
        # z_t = [0]
        self.min_angle = data.angle_min
        self.increment_angle = data.angle_increment

        for particle in self.particle_cloud:
            x = particle.pose.position.x
            y = particle.pose.position.y
            yaw = get_yaw_from_pose(particle.pose)

            q = 1

            for angle in z_t:
                z_tk = data.ranges[angle]

                if z_tk <= 3.5:
                    endpoint_x = int(x + z_tk * math.cos(yaw + (self.min_angle + angle * self.increment_angle)))
                    endpoint_y = int(y + z_tk * math.sin(yaw + (self.min_angle + angle * self.increment_angle)))

                    x_coor = int((endpoint_x - self.map_origin.position.x)/ self.map_resolution)
                    y_coor = int((endpoint_y - self.map_origin.position.y)/ self.map_resolution)

                    dist = 0
                    if (x_coor >= 384 or y_coor>= 384):
                        dist = -1
                    else:
                        dist = self.distance_matrix[x_coor, y_coor]
                    

                    if dist != -1:
                        prob = probability(dist, 2.0)
                        q *= prob
                    else:
                        q = q * 0.000001
            if q != 0:
                particle.w = q
            else:
                particle.w = 0.000001
                    

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        print('Step4: Updating particles with motion model')
        # Calculate the pose control by using current and previous pose
        delta_trans, delta_rot1, delta_rot2 = self.motion_model.calculate_pose_control(self.odom_pose_last_motion_update, self.odom_pose)

        # Update the pose for each particle
        for particle in self.particle_cloud:
            particle.pose = self.motion_model.sample_motion_model(delta_trans, delta_rot1, delta_rot2, self.motion_model.alpha, particle.pose)
            # print(particle.pose)
             

if __name__=="__main__":
    
    pf = ParticleFilter()
    rospy.spin()
    #rate = rospy.Rate(10)

    #while not rospy.is_shutdown():
        # Need to publish particle cloud and estimated robot pose continuously
        #if pf.particle_updated:
        #	pf.publish_particle_cloud()
        #	pf.publish_estimated_robot_pose()
        #	pf.particle_updated = False
        #rate.sleep









