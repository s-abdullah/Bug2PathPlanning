#!/usr/bin/env python

# roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=bug2-0.world

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
import math
from sensor_msgs.msg import LaserScan
import os

# initial vlaue of the global variable
global g_range_ahead
g_range_ahead = 100 # anything to start

global g_range_ahead
g_range_left = 100 # anything to start

global g_range_ahead
g_range_right = 100 # anything to start

global threshold
threshold = 0.9;


nanval = 100; 