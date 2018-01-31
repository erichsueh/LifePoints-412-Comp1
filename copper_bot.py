#!/usr/bin/env python

import roslib
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def find_object(ranges):
    rightSide = -1
    leftSide = -1
    size = len(ranges)
    i = 0
    while(math.isnan(ranges[i])):
        i = i + 1

    lastDist = ranges[i]
    i = i + 1

    while(i < size and rightSide == -1):    
        if not math.isnan(ranges[i]):
            if((ranges[i]-lastDist)> 0.3):
                rightSide = i
            else:
                lastDist = ranges[i]
        i = i + 1

    if(rightSide == -1):
        rightSide = 0

    i = size - 1
    while(math.isnan(ranges[i])):
        i = i - 1
    
    lastDist = ranges[i]
    i = i - 1
    
    while(i >= 0 and leftSide == -1):
        if not math.isnan(ranges[i]):
            if((ranges[i]-lastDist) > 0.3):
                leftSide = i
            else:
                lastDist = ranges[i]
        i = i - 1
    if(leftSide == -1):
        leftSide = size

    print rightSide
    print leftSide
    center = leftSide + rightSide
    if((center % 2) == 1):
        center = center + 1
    center = center/2
    return center



def scan_callback(msg):
    global cmd_vel_pub
    global old_vel
    bot_center = find_object(msg.ranges)
    #print bot_center
    angleAway = (len(msg.ranges)-bot_center-1)*msg.angle_increment+msg.angle_min
    if(abs(angleAway) < 0.01):
        angleAway = 0

    print(angleAway)
    vel = 0

    
    if not math.isnan(msg.ranges[bot_center]):
        difference = msg.ranges[bot_center] - 0.7
        if(difference < 0.05):
            difference = 0
        vel = difference/2
        if abs(vel-old_vel) > 0.5:
            vel = old_vel + (vel-old_vel)*0.8
        
    
    twist = Twist()
    twist.linear.x = vel
    old_vel = vel
    twist.angular.z = angleAway*-2
    cmd_vel_pub.publish(twist)



rospy.init_node('copper')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
global cmd_vel_pub
global old_vel
old_vel = 0
cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
rospy.spin()