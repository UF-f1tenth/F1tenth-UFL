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
filename = os.path.join(dirname, '../waypoints/fifthlab1.csv')
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

    #wrong?
    if (yaw[2] <= 0.785 and yaw[2] >= -0.785):
        direction = "A"
    elif (yaw[2] <= 2.355 and yaw[2] >= 0.785):
        direction = "D"
    elif (yaw[2] >= -2.355 and yaw[2] <= -0.785):
        direction = "B"
    else:
        direction = "C"

    print("direction", direction)
    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    for point in path_points:
        if (direction == "A" and point[0] > x_cord):
            if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                min = dist((point), (x_cord, y_cord))
                closest_point = point
        if (direction == "B" and point[1] < y_cord):
            if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                min = dist((point), (x_cord, y_cord))
                closest_point = point
        if (direction == "C" and point[0] < x_cord):
            if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                min = dist((point), (x_cord, y_cord))
                closest_point = point
        if (direction == "D" and point[1] > y_cord):
            if (dist((point), (x_cord, y_cord)) < min and dist((point), (x_cord, y_cord)) >= LOOKAHEAD_DISTANCE):
                min = dist((point), (x_cord, y_cord))
                closest_point = point

    # 3. Transform the goal point to vehicle coordinates.
    goal_point = closest_point
    print("goal_point:", goal_point)

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
    if (direction == "A"):
        angle = -(2 * ( y_cord - goal_point[1])) / (LOOKAHEAD_DISTANCE**2)
    elif(direction == "C"):
        angle = (2 * ( y_cord - goal_point[1])) / (LOOKAHEAD_DISTANCE**2)
    elif(direction == "D"):
        angle = (2 * ( x_cord - goal_point[0])) / (LOOKAHEAD_DISTANCE**2)
    else:
        angle = -(2 * ( x_cord - goal_point[0] )) / (LOOKAHEAD_DISTANCE**2)


    angle = np.clip(angle, -0.2967, 0.2967) # 0.2967 radians = 17 degrees because car can only turn 24 degrees max
    print("angle", angle)
    print("******************************************")

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

