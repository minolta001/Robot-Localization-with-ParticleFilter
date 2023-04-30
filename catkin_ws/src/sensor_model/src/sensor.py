import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from motion_model.msg import motion_model_msgs
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Header, String
import rospy
import random
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

        # motion 
        self.x = 0
        self.y = 0
        self.yaw = 0

        # map
        self.width = 0
        self.height = 0 
        rospy.wait_for_service("static_map") 
        house_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = house_map().map
        self.map_coordinates = []

        self.resolution = 0
        self.origin = 0

        self.dist_matrix = []
        #self.likelihood_matrix = []

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/motion_model', motion_model_msgs, self.motion_callback)

        self.run()

    
    def run(self):
        #grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
        self.width = self.map.info.width
        self.height = self.map.info.height
        self.resolution = self.map.info.resolution
        #self.origin = grid_msg.info.origin

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
        return 1 / (math.sqrt(2 * math.pi) * cov) * math.exp(-(x ** 2) / (2 * cov ** 2))
    

    # compute distance
    def distance(self, rx, ry, end_x, end_y):
        return ((rx - end_x) * self.resolution) ** 2 + ((ry - end_y) * self.resolution) ** 2



    def all_cells_go_through(self, rob_x, rob_y, end_x, end_y):
        dx = abs(end_x - rob_x)
        dy = abs(end_y - rob_y)
        sx = 1 if rob_x < end_x else -1
        sy = 1 if rob_y < end_y else -1
        err = dx - dy
        points = []
        
        while rob_x != end_x or rob_y != end_y:
            points.append((rob_x, rob_y))
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                rob_x += sx
            if e2 < dx:
                err += dx
                rob_y += sy
        points.append((rob_x, rob_y))
        return points
                          

    def likelihood_field(self):
        for i in range(self.width):
            for j in range(self.height):
                if(self.dist_matrix[i, j] != 0 and self.dist_matrix[i, j] != -1):
                    res = self.pdf_gaussian(self.dist_matrix[i, j], 0.2)
                    print(res)
                    self.likelihood_matrix[i, j] = res
        np.savetxt('likelihood_matrix_precise.csv', self.likelihood_matrix, delimiter=',')

         

    def likelihood_field_range_finder(self, x, y, yaw):
        read_interval = 20
        q = 1 
        for i in range(int(360 / read_interval)):
            scan_range = self.all_scans[i * read_interval - 1]
            #print(scan_range)
            if scan_range <= self.scan_max:      # suppose sensor is at the center of the robot 
                endpoint_x = x + scan_range * math.cos(yaw + (self.angle_min + i * read_interval * self.angle_increment))
                endpoint_y = y + scan_range * math.sin(yaw + (self.angle_min + i * read_interval * self.angle_increment))
                points = self.all_cells_go_through(int(x), int(y), int(endpoint_x), int(endpoint_y)) 

                #print(int((x / self.resolution) + int(self.width / 2)) , int((y / self.resolution) + int(self.width / 2)))

                for (px, py) in points:

                    px = int((px / self.resolution) + int(self.width / 2))
                    py = int((py / self.resolution) + int(self.height / 2))

                    dist_sq = self.dist_matrix[px, py]
                    print(dist_sq)
                    prob_hit = self.pdf_gaussian(dist_sq, 0.2)
                    
                    if(self.dist_matrix[px, py] != 0 and self.dist_matrix[px, py] != -1):
                        self.likelihood_matrix[px, py] = prob_hit  
                        #res.append(round(prob_hit, 2))


                #dist_sq, min_x, min_y = self.find_dist_min_pair(endpoint_x, endpoint_y)
                dist = self.dist_matrix[int(endpoint_x), int(endpoint_y)] 
                if dist == -1:
                    dist = 0
                q = q * (1 * self.pdf_gaussian(dist, 0.1))


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


class likelihood_field(object):
    def __init__(self):

        self.initialized = False

        rospy.init_node('likelihood_file')
        
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        rospy.Subscriber('scan', LaserScan, self.range_finder)
        
        self.particles_pub = rospy.Publisher('particels', PoseArray, queue_size=10)

        self.map = OccupancyGrid()
        self.sensor_model = sensor()
        
        self.particles_list = []  

        self.particles_initialize()
        self.initialized = True
        

    def particles_initialize(self):
        particles_num = 10
        for i in range(particles_num):
            x = random.uniform(-3.5, 3.5)
            y = random.uniform(-3.5, 3.5)
            theta = random.uniform(0, math.pi)
            
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

    def normalize_particles(self):
        weight_sum = 0
        for p in self.particles_list:
            weight_sum = weight_sum + p.weight
        
        for p in self.particles_list:
            p.weight = p.weight / weight_sum
    
    def map_callback(self, map_data):
        self.map = map_data
    
    def particles_publish(self):
        particles_pose_array = PoseArray()
        particles_pose_array.header = Header(stamp=rospy.Time.now(), frame_id='map')
        particles_pose_array.poses

        for par in self.particles_list:
            particles_pose_array.poses.append(par)

        self.particles_pub.publish(particles_pose_array)
    
    def range_finder(self, scan_data):
        if not self.initialized:
            return
        
        

if __name__ == '__main__':
    #sensor = sensor()
    #sensor.run()
    