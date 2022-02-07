#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 0.7 # meters
#VELOCITY = 1.0 # m/s
VELOCITY = 0.5


###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/watpoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(data):
    min = 5000000
    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
    x_cord = (data.pose.position.x)
    y_cord = (data.pose.position.y)
    yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    print("car x:", x_cord)
    print("car y:", y_cord)
    print("car yaw:", yaw[2])

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    pos = [x_cord, y_cord]
    yaw2 = yaw[2] 
    # 3. Transform the goal point to vehicle coordinates.
    des_angle = [round(yaw2 - 0.785, 3), round(yaw2 + 0.785, 3)]
    print("Desired Angle:", des_angle)

    #generating points a and b to create lines
    a = [round(math.cos(des_angle[0]) + pos[0], 3), round(math.cos(des_angle[1]) + pos[1], 3)]
    b = [round(math.sin(des_angle[0]) + pos[0], 3), round(math.sin(des_angle[1]) + pos[1], 3)]
    print("Generating point a:", a)
    print("Generating point b: " + str(b)  + "\n\n")

    #generating line a
    print("line a:")
    if(pos[0]-a[0] == 0):
        slop_a = None
        print("slope up")
        print("x = " + str(pos[0]) + "\n\n")
    else:
        slop_a = round((pos[1] - a[1]) / (pos[0] - a[0]), 2)
        b_a = round(a[1] - slop_a * a[0], 2)
        print("slop_a:", slop_a)
        print("b_a:", b_a)
        print("y = " + str(slop_a) + "x + " + str(b_a) + "\n\n")

    #generating line b
    print("line b:")
    if(pos[0]-b[0] == 0):
         slop_b = None
         print("slope up")
         print("x = " + str(pos[0]) + "\n\n")
    else:
        slop_b = round((pos[1] - b[1]) / (pos[0] - b[0]), 2)
        b_b = round(b[1] - slop_b * b[0], 2)
        print("slop_b:", slop_b)
        print("b_b:", b_b)
        print("y = " + str(slop_b) + "x + " + str(b_b) + "\n\n")

    #picking best point to follow
    for point in path_points:
        #determining direction
        if (yaw2 >= -0.785 and yaw2 <= 0.785):
            direction = "A"
            #evaluating slope
            if (slop_b == None):
                #checking if point is within bounds of both lines
                if (point[1] < (slop_a * point[0] + b_a) and point[0] > pos[0]):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            elif (slop_a == None):
                if (point[0] < pos[0] and point[1] > (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            else:
                if (point[1] < (slop_a * point[0] + b_a) and point[1] > (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
        elif(yaw2 < -0.785 and yaw2 >= -2.35):
            direction = "D"
            if (slop_b == None):
                if (point[1] < (slop_a * point[0] + b_a) and point[0] < pos[0]):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            elif (slop_a == None):
                if (point[0] < pos[0] and point[1] < (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            else:
                if (point[1] < (slop_a * point[0] + b_a) and point[1] < (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
        elif(yaw2 > 0.785 and yaw2 <= 2.35):
            direction = "B"
            if (slop_b == None):
                if (point[1] > (slop_a * point[0] + b_a) and point[0] > pos[0]):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            elif (slop_a == None):
                if (point[0] > pos[0] and point[1] > (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            else:
                if (point[1] > (slop_a * point[0] + b_a) and point[1] > (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
        else:
            direction = "C"
            if (slop_b == None):
                if (point[1] > (slop_a * point[0] + b_a) and point[0] < pos[0]):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point

            elif (slop_a == None):
                if (point[0] > pos[0] and point[1] < (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point
            else:
                if (point[1] > (slop_a * point[0] + b_a) and point[1] < (slop_b * point[0] + b_b)):
                    if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                        min = dist((point), (x_cord, y_cord))
                        closest_point = point

    goal_point = closest_point
    print("direction:", direction)
    print("goal_point:", goal_point)
    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    if (direction == "A"):
        angle = -(2 * ( y_cord - goal_point[1])) / (min**2)
    elif(direction == "C"):
        angle = (2 * ( y_cord - goal_point[1])) / (min**2)
    elif(direction == "D"):
        angle = (2 * ( x_cord - goal_point[0])) / (min**2)
    else:
        angle = -(2 * (x_cord - goal_point[0] )) / (min**2)


    angle = np.clip(angle, -0.2967, 0.2967) # 0.2967 radians = 17 degrees because car can only turn 24 degrees max
    print("######################################################################")

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

