#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math
from sensor_msgs.msg import Joy

def joy_callback(msg):
    global started
    if(msg.buttons[2]==1):
        started = True
    elif(msg.buttons[1]==True):
        started = False

def move(forward,turn):
    #print("the forward/turn are:",forward,turn)
    global cmd_vel_pub
    global currx
    global currz
    global started
    twist = Twist()
    if started == True:
        newx= calcx(forward)
        newz = calcz(turn)
        #print("newx and newz are:",newx,newz)
        twist.linear.x = newx
        twist.angular.z = newz
        cmd_vel_pub.publish(twist)

def calcz(forward):
    global currx
    #print("the fwd and curr x are: ",forward,currx)
    if (currx < forward):
        currx += 0.1
        return(currx)
    elif(currx > forward):
        currx -= 0.1
        return(currx)
    else:
        return(forward)

def calcx(turn):
    global currz
    #print("the turn and curr z are: ",turn,currz)
    if (currz < turn):
        currz +=0.1
        return(currz)
    elif(currz > turn):
        currz -=0.1
        return(currz)
    else:
        return(turn)

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = msg.ranges[len(msg.ranges)/2]
    
    g_side_ahead = 0
    while(math.isnan(msg.ranges[g_side_ahead])):
        g_side_ahead = g_side_ahead + 1

    if(msg.ranges[g_side_ahead] < 0.8):
        move(.8,1)
        
    print("the range ahead is: ", g_range_ahead)
    
class HardTurn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Forward','Turning','HardTurn'])

    def execute(self, userdata):
        global g_range_ahead
        global cmd_vel_pub
        global state_change_time
        global driving_forward
        global rate
        rospy.loginfo('Executing state HardTurn')
        '''
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 1
        cmd_vel_pub.publish(twist)
        '''
        move(0,1)
        rate.sleep()
        print(g_range_ahead)
        if(g_range_ahead < .7):
            return 'HardTurn'
        elif(g_range_ahead < 2 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(2)
            return 'Turning'
        else:
            return 'Forward'
            
#define state Forward
class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Forward', 'Turning','HardTurn'])

    def execute(self, userdata):
        global g_range_ahead
        global cmd_vel_pub
        global state_change_time
        global driving_forward
        global rate
        
        rospy.loginfo('Executing state Forward')
        '''
        twist = Twist()
        twist.linear.x = 1
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)
        '''
        
        move(.8,-.2)
        rate.sleep()
        if(g_range_ahead < .7):
            return 'HardTurn'
        elif(g_range_ahead < 2 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(2)
            return 'Turning'
        else:
            return 'Forward'


class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Forward','Turning','HardTurn'])

    def execute(self, userdata):
        global cmd_vel_pub
        global state_change_time
        global driving_forward
        global rate
        global g_range_ahead
        rospy.loginfo('Executing state Turning')
        '''
        twist = Twist()
        twist.linear.x = 1
        twist.angular.z = .5
        cmd_vel_pub.publish(twist)
        '''
        move(.6,1.5)
        rate.sleep()
        if(g_range_ahead < .7):
            return 'HardTurn'
        elif(g_range_ahead < 2 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(2)
            return 'Turning'
        else:
            return 'Forward'



def main():
    rospy.init_node('wander')

    #Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    #Open the container
    with sm:
        #Add states to the container
        smach.StateMachine.add('Forward', Forward(), transitions = {'Forward':'Forward','Turning':'Turning','HardTurn':'HardTurn'})
        smach.StateMachine.add('Turning', Turning(), transitions = {'Forward':'Forward', 'Turning':'Turning','HardTurn':'HardTurn'})
        smach.StateMachine.add('HardTurn', HardTurn(), transitions = {'Forward':'Forward', 'Turning':'Turning','HardTurn':'HardTurn'})
    sis = smach_ros.IntrospectionServer('wanderSM', sm, '/SM_ROOT')
    sis.start()
    global g_range_ahead
    global scan_sub
    global state_change_time
    global driving_forward
    global rate
    global currx
    global currz
    global started
    started = False
    currx = 0
    currz = 0
    g_range_ahead = 1
    joy_sub = rospy.Subscriber('joy',Joy, joy_callback)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    state_change_time = rospy.Time.now()
    driving_forward = True
    rate = rospy.Rate(10)
    
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()
