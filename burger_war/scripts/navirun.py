#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


    def clearGoal(self):
        self.client.cancel_all_goals()
        print 'stop_order'
        return

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return -1
        get_state = self.client.get_state()
        print("wait", wait, "get_state", get_state)
        if get_state == 2:  # if send_goal is canceled
            return -1

        return 0       


    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        '''self.setGoal(-0.8,0.6,3.1415/4)
        self.setGoal(-0.6,0.8,3.1415/4)
        '''
        self.setGoal(-0.9,0.4,0)
        self.setGoal(-0.9,0.4,-3.1415/2)

        self.clearGoal()

        self.setGoal(-0.9,-0.4,0)
        self.setGoal(-0.9,-0.4,3.1415/2)

        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415/2)
        #my area ^
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,-3.1415/2)
        self.setGoal(0,0.5,3.1415)
        #left area ^
        
        self.setGoal(-0.5,0,-3.1415/2)
        #bips L2R
        
        self.setGoal(0,-0.5,0)
        self.setGoal(0,-0.5,3.1415/2)
        self.setGoal(0,-0.5,3.1415)
        #right area ^

        self.setGoal(0.5,0,3.1415/2)
        #bips R2T


if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()