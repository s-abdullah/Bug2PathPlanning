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




class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

        # How fast will we update the robot's movement?
        self.rate = 20

        self.follow = "line"

        self.orient = 0;

        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        self.linear_speed = 0.15
        
        # Set the travel distance in meters
        self.goal_distance = 10


        # Set the rotation speed in radians per second
        self.angular_speed = 0.8
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(0.8)
        
        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = pi

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        self.position = Point()
        self.init = Point()
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        
        # Get the starting position values     
        (self.position, self.rotation) = self.get_odom()
                    
        self.init.x = self.position.x
        self.init.y = self.position.y
        # print "The positon is:", x_start, y_start
        
        # Keep track of the distance traveled
        self.distance = 0

        # skipping variable
        self.skip = 0;
        
        # Enter the loop to move along a side
        while self.distance < self.goal_distance and not rospy.is_shutdown():
            self.printer()

            if (g_range_left < threshold):
                if self.getFollow() == "line":
                    self.stop();
                    self.change();

                    while (self.getFollow() == "object"):
                        self.skip = 0;
                        print "object detection mode"
                        self.printer()
                        # rotate 90 to see if there is object

                        # rotate 90 and go the distance that is seen
                        # keep of ratating until no object infront
                        prevL = 0.0;
                        prevR = 0.0;
                        prevA = 0.0;


                        while (g_range_left < 100)&(g_range_ahead < 100):
                            self.skip = 1;
                            self.printer()
                            # for sucessive iterations may get away from the wall
                            if self.getFollow() == 'line':
                                break;
                            if (g_range_left > 1.2)&(g_range_ahead > 1.2):
                                # move the distance measured 
                                print "moving forward to get close"
                                self.move(0.2)
                                self.stop()
                                # this also means that the angles are skewd so we adjust for that
                                self.rotate(-1*pi/72, -1*angular_tolerance);
                                self.stop()
                                


                            print "object wall encountrered, turning right"
                            if (g_range_left == 100)&(g_range_ahead == 100):
                                break;

                            prevR = g_range_right;
                            if g_range_left < 100:
                                prevL = g_range_left;
                            if g_range_ahead < 100:
                                prevA = g_range_ahead;

                            cuts = 4
                            for x in range(cuts):
                                self.printer()

                                if prevL > g_range_left:
                                    prevL = g_range_left;
                                if prevA > g_range_ahead:
                                    prevA = g_range_ahead;
                                # rotate to get parallel to the wall
                                self.rotate(pi/2, angular_tolerance);
                                # move the distance measured 
                                print "moving right for the observed distance"
                                self.move(self.dist(prevL/cuts, prevA/cuts, 1))
                                self.stop()

                                # checking to see of there is a wall here aswell
                                if (g_range_left < threshold)&(g_range_ahead < threshold):
                                    # if there is a wall then add thre to the orient and skipp all the this shit
                                    print "another wall infront of me"
                                    print "moving alittle forward to get close"
                                    self.move(0.1)
                                    self.stop()
                                    
                                    self.orient = self.orient + 3;
                                    break;

                                # move back to the wall
                                print "turning back around"
                                self.rotate(-1*pi/2, -1*angular_tolerance);
                                self.stop()
                                if self.checker(1):
                                    break;
                                    # braking
                            #     if (g_range_left == 100)&(g_range_ahead == 100):
                            #         break;
                            # if (g_range_left == 100)&(g_range_ahead == 100):
                            #     break;
                            # # self.printer()
                        if self.checker(1):
                            break;

                            
                        self.printer()
                        self.stop()

                                

                        if (g_range_right < 1.2)&(g_range_left == 100)&(g_range_ahead <= 100):
                            self.skip = 1;
                            print "wall on the side, avoiding it and moving away "
                            prevR = g_range_right;
                            self.rotate(pi/2, angular_tolerance);
                            self.move(self.dist(prevR/2.5, prevR/2.5, 1))
                            self.rotate(-1*pi/2, -1*angular_tolerance);
                            self.stop()


                        if self.checker(1):
                            break;

                        self.printer()
                        if (g_range_left >= 3.5)&(g_range_ahead == 100):
                            self.skip = 1;
                            # if g_range_right < 0.4:
                            #     self.rotate(pi/6, angular_tolerance);
                            if self.getFollow() == 'object':
                                print "nothing ahead so i head forward and find the next wall"
                                self.orient = self.orient + 1;
                                print self.orient
                                if prevL <= 0.15:
                                    self.move(0.6)
                                else:
                                    self.move(prevL)
                                self.rotate(-1*pi/2, -1*angular_tolerance);

                        # if no conditions are bing me then move it move it
                        if self.skip == 0:
                            self.move(0.1)
                            self.stop()
                        self.checker(1);
                        self.printer()


            else:
                # follow the m line here
                # until object is hit

                move_cmd.linear.x = self.linear_speed                    

                # # # Publish the Twist message and sleep 1 cycle         
                self.cmd_vel.publish(move_cmd)
                
                self.r.sleep()
        
                # # Get the current position
                (self.position, self.rotation) = self.get_odom()
                
                # Compute the Euclidean distance from the start
                self.checker(2);
                distance = sqrt(pow((self.position.x - self.init.x), 2) + 
                                pow((self.position.y - self.init.y), 2))

            
        # Stop the robot for good
        self.stop()
        

    def checker(self, tog):
        thr = 0.1
        if tog == 1:
            pass
            print "checker"
            print self.getFollow();
            # print "x: " , self.position.x
            # print "y: " , self.position.y
            (self.position, self.rotation) = self.get_odom()


            if abs(self.position.y) < thr*2:
                print "On the m-line"
                if (abs(self.position.x - self.goal_distance) < thr):
                    print "Goal Reached"
                    self.stop();
                    # bad stopping condition
                    self.shutdown();
                    os._exit(0)
                    
                    # assert 1==2   
                elif self.position.x > self.init.x:
                    print "closer to the m-line"
                    if self.follow != 'line':
                        print "rotating for the m-line"
                        self.change()
                        self.rotate(((pi/2)*(self.orient%4)-(pi/36)), radians(0.8));
                        self.orient = 0;
                        return 1;
                elif abs(self.position.x - self.init.x) < thr:
                    print "Not gonna reach, Bye Bye"
                    self.shutdown();
                    os._exit(0)
            return 0;

        if tog == 2:
            if (abs(self.position.x - self.goal_distance) <= thr)&(abs(self.position.y) <= thr*15):
                print "Goal Reached"
                self.stop();
                # realligning the y direction
                if abs(self.position.y) > thr:
                    print "re-alligning the Y direction due to noise"
                    if self.position.y < 0:
                        self.rotate((pi/2), radians(1.0));
                        self.move(abs(self.position.y))
                    else:
                        self.rotate((-1*(pi/2)), -1*radians(1.0));
                        self.move(abs(self.position.y))

                self.shutdown();
                os._exit(0)

            return 0;
                    # bad stopping condition
                    # assert 1==2   
        # elif tog ==2;

    # def checkMove(self):


    def dist(self, val1, val2, tog):
        if tog == 1:
            return math.sqrt(pow(val1,2)+pow(val2,2));
        elif tog == 2:
            return math.sin(0.52)*val1;


    def rotate(self, angle, tolerance):
        (self.position, self.rotation) = self.get_odom()
        move_cmd = Twist()
        # Set the movement command to a rotation
        if angle > 0:
            move_cmd.angular.z = self.angular_speed
        else:
            move_cmd.angular.z = -1*self.angular_speed
            angle = -1*angle
            
        # Track the last angle measured
        last_angle = self.rotation
        
        # Track how far we have turned
        turn_angle = 0
        while abs(turn_angle + tolerance) < abs(angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            
            # Get the current rotation
            (self.position, self.rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(self.rotation - last_angle)
            
            # Add to the running total
            turn_angle += delta_angle
            last_angle = self.rotation


    def move(self, dist):
        
        # How long should it take us to get there?
        linear_duration = dist / self.linear_speed
        
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the forward speed
        move_cmd.linear.x = self.linear_speed
        
        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * self.rate)
        
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()


    def stop(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def printer(self):
        print "the rangeLeft is: ",g_range_left
        print "the rangeAhead is: ",g_range_ahead
        print "the rangeRight is: ",g_range_right
        print "***************"
        print "x: " , self.position.x
        print "y: " , self.position.y


    def change(self):
        if self.follow == "line":
            self.follow = "object"
            print "encountrered object"
        else:
            self.follow = "line";
            print "following m-line"
    def getFollow(self):
        return self.follow;


    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 

if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("terminated.")