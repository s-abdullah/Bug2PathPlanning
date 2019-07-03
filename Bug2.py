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



def scan_callback(msg):
    # global variable that holds the range of the object ahead
    # print "min angle: ", msg.angle_min
    # print "max angle: ", msg.angle_max
    # print "min range: ", msg.range_min
    # print "max range: ", msg.range_max
    # print "ranges",len(msg.ranges)
    # print "the rangeAhead is: ",g_range_ahead
    # print "the rangeLeft is: ",g_range_left
    # print "the rangeRight is: ",g_range_right


    # print msg.ranges

    cleanedList = list(msg.ranges)
    for x in range(len(msg.ranges)):
        if math.isnan(msg.ranges[x]):
            cleanedList[x] = nanval
            # voluntary blinditude
        elif cleanedList[x] >= 3.2:
            cleanedList[x] = nanval;


    global g_range_left
    g_range_left = min(cleanedList[539:639])

    global g_range_right
    g_range_right = min(cleanedList[:99])

    # smoothers
    if (g_range_right < 100)&(g_range_left < 100)&(min(cleanedList[269:369]) == 100):
        global g_range_ahead
        g_range_ahead = (g_range_right+ g_range_left)/4

    else:
        global g_range_ahead
        g_range_ahead = min(cleanedList[269:369])





if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("terminated.")