#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import copy

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 0.9 # meters
LOOKAHEAD_DISTANCE_OB = 0.5
#VELOCITY = 1.0 # m/s
VELOCITY = 0.4


###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/fifthlab1.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)



gap_len = 0
last_goal_point = []
car_width = 0.03
global gp_x
global gp_y

#############
# FUNCTIONS #
#############
class Server:
    def __init__(self):
        self.crash = 0
        self.orient = 'c'
        self.obst_idx = 0 #540
        self.dist_fw = 1.0
        self.obst_flag = False
    def getRange(self, data, theta):

	    if math.isnan(theta) or math.isinf(theta):
		    print ('encountered invalid value in getRange')
		    theta = 0.0
	    if theta < 0.0: theta = 0.0
	    if theta > 180.0: theta = 180.0

	    idx_float = ((theta+45.0) / 270.0) * (len(data.ranges) - 1)
	    idx = int(round(idx_float))
	    ret = data.ranges[idx]
	    return ret,idx if not math.isnan(ret) and not math.isinf(ret) else 3.0

    def find_gap(self, arr):                
                i = 180
                start_p = 0
                count  = 0
                last_max = 0
                start_id = 0
                global gap_len
                while(i<901 and i<len(arr)):
                        if arr[i] > LOOKAHEAD_DISTANCE_OB:
                            start_p = i
                            while arr[i]>LOOKAHEAD_DISTANCE_OB and i<901 and i<len(arr):
                                count+=1
                                i+=1
                            if count > last_max:
                                last_max = count
                                start_id = start_p
                        else:
                          count  = 0      
                          i+=1
                
                
                gap_len = last_max   
	            
                #print("start id:", start_id, last_max )
                if start_id < 540:
                        self.orient = 'r' 
                        self.obst_idx = start_id+last_max+2 #added 2 instead of 1 to avoid abrupt value from object edge
                elif start_id >= 540:
                        self.orient = 'l'
                        self.obst_idx = start_id-2
                else:
                        self.orient = 'c'
                        self.obst_idx = 0
                #print("ornt:", self.orient)

    def scan_callback(self, data):
        #self.data = data
        #print("range:", len(data.ranges))
        check_gap = 0
        new_arr = copy.deepcopy(list(data.ranges))        
        c, id2 = self.getRange(data,90)
        d, id1 = self.getRange(data,80)
        e, id3 = self.getRange(data,100)
        #print("value at 80deg:", d," at", id1)
        #print("value at 90deg:", c," at", id2)
        #print("value at 100deg:", e," at", id3)
        if(c <= 0.2):
            self.crash = 1
        else:
            if(d<=LOOKAHEAD_DISTANCE_OB):
                for i in range(480, 521):
                        new_arr[i]=0
                        check_gap = 1
            if(c<=LOOKAHEAD_DISTANCE_OB):
                for i in range(520, 561):
                        new_arr[i]=0
                        check_gap = 1
            if(e<=LOOKAHEAD_DISTANCE_OB):
                for i in range(560, 601):
                        new_arr[i]=0
                        check_gap = 1
            if check_gap == 1:
                self.crash = 1
                self.find_gap(new_arr)
                self.dist_fw = data.ranges[self.obst_idx]
            else:
                self.orient = 'c'
                self.dist_fw = 1.0
            self.crash = 0
        
        
        """
        if(c<=1.0):

		    front_error=0.45
	    else:

		    front_error=0

	    swing = math.radians(theta)

	    alpha = math.atan((a * math.cos(swing)-b) / (a * math.sin(swing)))
	    dist_AB = b * math.cos(alpha)

	    dist_AC = vel1 * 0.1
	    dist_CD = dist_AB + dist_AC * math.sin(alpha)
	    error = desired_distance - dist_CD +  front_error
	    print('\r','Dist theta: %f \t Dist 0: %f' %(a,b),end = " ")
	    print('\t car distance: ', dist_CD)
	    if math.isnan(error):
		    print ('nan occured:', a, b)

	    vel = base_vel * 1.0 / (vel_scale * abs(error) + 1)
	    return (-error), vel
        """

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self, data):
        #print("crash:", self.crash)
        if(self.crash == 0):
            min_val = 5000000
            closest_point = []
            global last_goal_point
            # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

            # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
            # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
            x_cord = (data.pose.position.x)
            y_cord = (data.pose.position.y)
            yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            #print("car x:", x_cord)
            #print("car y:", y_cord)
            #print("car yaw:", yaw[2])

            #wrong?
            if (yaw[2] <= 0.785 and yaw[2] >= -0.785):
                direction = "A"
            elif (yaw[2] <= 2.355 and yaw[2] >= 0.785):
                direction = "D"
            elif (yaw[2] >= -2.355 and yaw[2] <= -0.785):
                direction = "B"
            else:
                direction = "C"

            #print("direction", direction)
            # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
            for point in path_points:
                if (direction == "A" and point[0] > x_cord):
                    if (self.dist((point), (x_cord, y_cord)) < min_val and self.dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min_val = self.dist((point), (x_cord, y_cord))
                        closest_point = list(point)
                if (direction == "B" and point[1] < y_cord):
                    if (self.dist((point), (x_cord, y_cord)) < min_val and self.dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min_val = self.dist((point), (x_cord, y_cord))
                        closest_point = list(point)
                if (direction == "C" and point[0] < x_cord):
                    if (self.dist((point), (x_cord, y_cord)) < min_val and self.dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min_val = self.dist((point), (x_cord, y_cord))
                        closest_point = list(point)
                if (direction == "D" and point[1] > y_cord):
                    if (self.dist((point), (x_cord, y_cord)) < min_val and self.dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min_val = self.dist((point), (x_cord, y_cord))
                        closest_point = list(point)

            # 3. Transform the goal point to vehicle coordinates.
            if not closest_point:
                    closest_point = last_goal_point
                    
            goal_point = closest_point
            print("cp:", closest_point)           
            global car_width
            global gp_x
            global gp_y

            if self.obst_flag == False:
                    gp_x = goal_point[0]
                    gp_y = goal_point[1]
                    if self.orient == 'r' or self.orient == 'l':
                        self.obst_flag = True                    
                        fw = self.dist_fw
                        shift_dist = np.sqrt(float(fw**2) - float(LOOKAHEAD_DISTANCE_OB**2)) + 0.2
                        if (self.orient == 'r'):
                            print("turn right")                   
                            if (direction == "A"):
                                gp_x = x_cord + LOOKAHEAD_DISTANCE_OB                          
                                gp_y = y_cord - shift_dist 
                            elif(direction == "C"):
                                gp_x = x_cord - LOOKAHEAD_DISTANCE_OB                          
                                gp_y = y_cord + shift_dist 
                            elif(direction == "D"):
                                gp_x = x_cord + shift_dist                        
                                gp_y = y_cord + LOOKAHEAD_DISTANCE_OB 
                            else:
                                gp_x = x_cord - shift_dist                        
                                gp_y = y_cord - LOOKAHEAD_DISTANCE_OB   
                        else:
                            print("turn left")              
                            if (direction == "A"):
                                gp_x = x_cord + LOOKAHEAD_DISTANCE_OB                          
                                gp_y = y_cord + shift_dist 
                            elif(direction == "C"):
                                gp_x = x_cord - LOOKAHEAD_DISTANCE_OB                          
                                gp_y = y_cord - shift_dist 
                            elif(direction == "D"):
                                gp_x = x_cord - shift_dist                        
                                gp_y = y_cord + LOOKAHEAD_DISTANCE_OB 
                            else:
                                gp_x = x_cord + shift_dist                        
                                gp_y = y_cord - LOOKAHEAD_DISTANCE_OB 
            else:
                    if (direction == "A"):
                        if(x_cord >= (gp_x-0.1)):
                            self.obst_flag = False
                            self.orient = 'c'
                            gp_x = goal_point[0]
                            gp_y = goal_point[1]
                    elif(direction == "C"):
                        if(x_cord <= (gp_x+0.1)):
                            self.obst_flag = False
                            self.orient = 'c'
                            gp_x = goal_point[0]
                            gp_y = goal_point[1]
                    elif(direction == "D"):
                        if(y_cord >= (gp_y-0.1)):
                            self.obst_flag = False
                            self.orient = 'c'
                            gp_x = goal_point[0]
                            gp_y = goal_point[1]
                    else:    
                        if(y_cord <= (gp_y+0.1)):
                            self.obst_flag = False
                            self.orient = 'c'
                            gp_x = goal_point[0]
                            gp_y = goal_point[1]   

            if (direction == "A"):
                angle = -(2 * ( y_cord - gp_y)) / (LOOKAHEAD_DISTANCE**2)
            elif(direction == "C"):
                angle = (2 * ( y_cord - gp_y)) / (LOOKAHEAD_DISTANCE**2)
            elif(direction == "D"):
                angle = (2 * ( x_cord - gp_x)) / (LOOKAHEAD_DISTANCE**2)
            else:
                angle = -(2 * ( x_cord - gp_x )) / (LOOKAHEAD_DISTANCE**2)  
            '''
            #print("x, y", gp_x, gp_y)         

            if (direction == "A"):
                if self.orient == 'r':
                   gp_y = gp_y-0.20 
                elif self.orient == 'l':
                   gp_y = gp_y+0.20
            elif(direction == "C"):
                if self.orient == 'r':
                   gp_y = gp_y+0.20 
                elif self.orient == 'l':
                   gp_y = gp_y-0.20                
            elif(direction == "D"):
                if self.orient == 'r':
                   gp_x = gp_x+0.20 
                elif self.orient == 'l':
                   gp_x = gp_x-0.20                
            else:
                if self.orient == 'r':
                   gp_x = gp_x-0.20 
                elif self.orient == 'l':
                   gp_x = gp_x+0.20'''

            
            #print("orient:", self.orient)
            #print("x, y", gp_x, gp_y)
            #self.orient = 'c'
            #self.obst_flag = False
            last_goal_point = goal_point
            #print("goal_point:", goal_point)
            

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            """if (direction == "A"):
                angle = -(2 * ( y_cord - gp_y)) / (LOOKAHEAD_DISTANCE**2)
            elif(direction == "C"):
                angle = (2 * ( y_cord - gp_y)) / (LOOKAHEAD_DISTANCE**2)
            elif(direction == "D"):
                angle = (2 * ( x_cord - gp_x)) / (LOOKAHEAD_DISTANCE**2)
            else:
                angle = -(2 * ( x_cord - gp_x )) / ww(LOOKAHEAD_DISTANCE**2)"""


            angle = np.clip(angle, -0.2967, 0.2967) # 0.2967 radians = 17 degrees because car can only turn 24 degrees max
            #print("angle", angle)
            #print("******************************************")
            #print("wanna drive\n")

            msg = drive_param()
            msg.velocity = VELOCITY
            msg.angle = angle
            pub.publish(msg)
        else:
            #print("stop me\n")
            msg = drive_param()
            msg.velocity = 0
            msg.angle = 0
            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')

    server = Server()

    rospy.Subscriber("scan", LaserScan, server.scan_callback)
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, server.callback, queue_size=1)
    rospy.spin()

