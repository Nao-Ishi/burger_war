#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random


from geometry_msgs.msg import Twist

import tf

from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import math
from nav_msgs.msg import Odometry

from enum import Enum

class STATE(Enum):
    START = 1
    SNEAK = 2
    TRACK = 3
    PATROL = 4

class NaoBot():
    def __init__(self):
        # waypoints loader from csv
        #------------------------
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.get_targetFlg=False

        # war state subscriber
        rospy.Subscriber("war_state",String,self.war_stateCb)
        self.pre_field_score = ['n'] * 18 
        self.field_score = ['n'] * 18

        # target id subscriber #...dont use?
        rospy.Subscriber("target_id",MarkerArray,self.target_idCb)

        # odometry subscriber
        rospy.Subscriber("odom",Odometry,self.odomCb)

        # game status
        self.N_EXISTS = True
        self.pre_time = rospy.Time.now().to_sec()
        self.pre_state = STATE.START
        self.area=[1,2,4,3]
        self.my_pos=1

        self.tour_pnt = 0
        self.get_id = 0
        self.enemy_policy_est = False
        self.cw = 1
        self.enemy_log = 0
        self.pre_enemy_log = 0
        self.enemy_pos = 4
        self.pre_enemy_pos = 4
        self.estimate_enemy_pos = 3

        # get rosparam
        # side
        self.my_side = rospy.get_param('~side')
        if self.my_side == 'r':
            self.enemy_side = 'b'
        else:
            self.enemy_side = 'r'
        print "mybot",self.my_side,"enemybot",self.enemy_side

        # waypoint list
        if self.my_side == 'r':
            self.goal =[#side:=r #x,y,deg #target id
                [0.8,0.45,170],  #6
                [0,0.5,0],      #7
                [0.8,-0.45,-170],#8
                [0,-0.5,0],     #9
                [0,0.5,180],    #10
                [-0.8,0.45,10],  #11
                [0,-0.5,180],   #12
                [-0.8,-0.45,-10],#13
                [0.5,0,180],    #14
                [0,-0.5,90],    #15
                [0,0.5,-90],    #16
                [-0.5,0,0],     #17
                [-0.8,0.45,-90],#myl2myr#18
                [-0.8,-0.45,90],#myr2myl#19
                [0.8,0.45,-90],#enr2enl#20
                [0.8,-0.45,90],#enl2enr#21
                [-0.5,0,90],#1to2#22
                [0,0.5,0],#2to4#23
                [0.5,0,-90],#4to3#24
                [0,-0.5,180],#3to1#25
                [-0.5,0,180],#my#26
                [0.5,0,0],#en#27
                [-1.2,0,0],#sneak1#28
                [0,1.2,-90],#sneak2#29
                [0,-1.2,90],#sneak3#30
                [1.2,0,180],#sneak4#31
                [-0.5,0,-90],#2to1#32
                [0,0.5,180],#4to2#33
                [0.5,0,90],#3to4#34
                [0,-0.5,0]#1to3#35

            ]
        else:
            self.goal =[#side:=r #x,y,deg #target id
                [-0.8,-0.45,-10],#6
                [0,-0.5,180],   #7
                [-0.8,0.45,10],  #8
                [0,0.5,180],    #9
                [0,-0.5,0],     #10
                [0.8,-0.45,-170],#11
                [0,0.5,0],      #12
                [0.8,0.45,170],  #13
                [-0.5,0,0],     #14
                [0,0.5,-90],    #15
                [0,-0.5,90],    #16
                [0.5,0,180],     #17   
                [0.8,-0.45,90],#myl2myr#18
                [0.8,0.45,-90],#myr2myl#19
                [-0.8,-0.45,90],#enr2enl#20
                [-0.8,0.45,-90],#enl2enr#21
                [0.5,0,-90],#1to2#22
                [0,-0.5,180],#2to4#23
                [-0.5,0,90],#4to3#24
                [0,0.5,0],#3to1#25
                [0.5,0,0],#my#26
                [-0.5,0,180],#en#27
                [-1.2,0,0],#sneak1#28
                [0,1.2,-90],#sneak2#29
                [0,-1.2,90],#sneak3#30
                [1.2,0,180],#sneak4#31
                [0.5,0,90],#2to1#32
                [0,-0.5,0],#4to2#33
                [-0.5,0,-90],#3to4#34
                [0,0.5,180]#1to3#35
            ]


    def war_stateCb(self,data):# war state subscriber #state transition trigger
        # unko code uncode
        # should use JSON protocol
        log = str(data.data).split('\n')
        pnt=range(17,103,5)
        n=0
        for i in pnt:
            self.field_score[n]=log[i][17]
            if self.pre_field_score[n] != self.field_score[n]:
                if self.field_score[n]==self.enemy_side:# n point -> enemypoint
                    print "n to ",self.enemy_side
                    self.enemy_log=n

                if self.field_score[n]==self.my_side and self.get_id==n:# get a target before reaching goal
                    self.clearGoal()# stop navigation
                    self.get_id=0# flg clear
                print "war state update id:",n,self.pre_field_score[n],"->",self.field_score[n]
                self.pre_field_score[n]=self.field_score[n]
            #print n,self.field_score[n]#debug
            n = n+1
        #print self.field_score#debug
        return

    def target_idCb(self,data):# target id subscriber
        log = str(data.markers[0]).split()
        self.target_id = log[11]
        self.get_targetFlg = True
        #self.clearGoal()
        #print self.target_id

    def odomCb(self,data):
        pose_x = data.pose.pose.position.x
        pose_y = data.pose.pose.position.y
        if self.my_side=='r':
            if pose_y < pose_x and pose_y < pose_x*(-1):
                self.my_pos=1
            elif pose_y > pose_x and pose_y < pose_x*(-1):
                self.my_pos=2
            elif pose_y < pose_x and pose_y > pose_x*(-1):
                self.my_pos=3
            elif pose_y > pose_x and pose_y > pose_x*(-1):
                self.my_pos=4
        else:
            if pose_y < pose_x and pose_y < pose_x*(-1):
                self.my_pos=4
            elif pose_y > pose_x and pose_y < pose_x*(-1):
                self.my_pos=3
            elif pose_y < pose_x and pose_y > pose_x*(-1):
                self.my_pos=2
            elif pose_y > pose_x and pose_y > pose_x*(-1):
                self.my_pos=1
            


    def enemy_policy(self):# enemy policy guesser
        
        if self.pre_enemy_log != self.enemy_log:
            self.pre_enemy_pos = self.enemy_pos
            #self.pre_time = rospy.Time.now().to_sec()
            if self.my_side=='r':
                if self.enemy_log in [6,8,14]:
                    print "enemy in area4"
                    self.enemy_pos = 4
                if self.enemy_log in [9,12,15]:
                    print "enemy in area3"
                    self.enemy_pos = 3
                if self.enemy_log in [7,10,16]:
                    print "enemy in area2"
                    self.enemy_pos = 2
                if self.enemy_log in [11,13,17]:
                    print "enemy in area1"
                    self.enemy_pos = 1
            else:
                if self.enemy_log in [6,8,14]:
                    print "enemy in area4"
                    self.enemy_pos = 1
                if self.enemy_log in [9,12,15]:
                    print "enemy in area3"
                    self.enemy_pos = 2
                if self.enemy_log in [7,10,16]:
                    print "enemy in area2"
                    self.enemy_pos = 3
                if self.enemy_log in [11,13,17]:
                    print "enemy in area1"
                    self.enemy_pos = 4
        if self.pre_enemy_pos != self.enemy_pos:
            self.pre_enemy_pos = self.enemy_pos
            for i in range(4):
                if self.pre_enemy_pos == self.area[i]:
                    self.area_pnt = i

            if self.estimate_enemy_pos == self.pre_enemy_pos:
                print "Guessing is correct"
                self.enemy_policy_est = True
            else:
                print "Guessing is incorrect"
                self.cw = self.cw * (-1)
                self.enemy_policy_est = False
            
            self.area_pnt = self.area_pnt + 1 * self.cw
            if self.area_pnt > 3:
                self.area_pnt = 0
            elif self.area_pnt < 0:
                self.area_pnt = 3
            self.estimate_enemy_pos = self.area[self.area_pnt]

          


    def get_target(self,id):
        self.get_id=id
        if self.get_id >= 6 :
            self.setGoal(self.goal[self.get_id-6])
        else:
            return 0

    #state
    def state_patrol(self):
        if self.pre_state != STATE.PATROL:
            self.clearGoal()
            if self.my_pos == 1:
                self.tour_pnt = 5
            elif self.my_pos == 2:
                self.tour_pnt = 6
            elif self.my_pos == 4:
                self.tour_pnt = 11
            else:
                self.tour_pnt = 19
        self.pre_state = STATE.PATROL
        if self.my_side == 'r':
            if self.cw == 1:
                tour=[11,18,13,19,17,22,23,7,16,10,23,24,27,8,21,6,20,14,24,25,12,15,9,25,22,26]
            else:
                tour=[13,19,11,18,17,32,35,9,15,12,35,34,27,6,20,8,21,14,34,33,10,16,7,33,32,26]
        else:
            if self.cw == 1:
                tour=[8,21,6,20,14,24,25,12,15,9,25,22,26,11,18,13,19,17,22,23,7,16,10,23,24,27]
            else:
                tour=[6,20,8,21,14,34,33,10,16,7,33,32,26,13,19,11,18,17,32,35,9,15,12,35,34,27]
        self.get_target(tour[self.tour_pnt])
        self.tour_pnt = self.tour_pnt + 1
        if self.tour_pnt > len(tour)-1:
            self.tour_pnt = 0
        
    def state_sneak(self):
        if self.pre_state != STATE.SNEAK:
            self.clearGoal()
            if self.my_pos == 1:
                self.get_target(28)
            elif self.my_pos == 2:
                self.get_target(29)
            elif self.my_pos == 3:
                self.get_target(30)
            else:
                self.get_target(31)
        self.pre_state = STATE.SNEAK

    def state_track(self):
        if self.pre_state != STATE.TRACK:
            self.clearGoal()
        if self.estimate_enemy_pos == self.my_pos:
            if self.cw == 1:
                if self.my_pos==1:
                    self.get_target(23)
                    self.get_target(24)
                    self.get_target(25)
                elif self.my_pos==2:
                    self.get_target(24)
                    self.get_target(25)
                    self.get_target(22)
                elif self.my_pos==4:
                    self.get_target(25)
                    self.get_target(22)
                    self.get_target(23)
                else:
                    self.get_target(22)
                    self.get_target(23)
                    self.get_target(24)
            else:
                if self.my_pos==1:
                    self.get_target(35)
                    self.get_target(34)
                    self.get_target(33)
                elif self.my_pos==2:
                    self.get_target(32)
                    self.get_target(35)
                    self.get_target(34)
                elif self.my_pos==4:
                    self.get_target(33)
                    self.get_target(32)
                    self.get_target(35)
                else:
                    self.get_target(34)
                    self.get_target(33)
                    self.get_target(32)
                    


    #---------------------------#



    def clearGoal(self):
        self.client.cancel_all_goals()
        print 'stop_order'
        return 0

    def setGoal(self,waypoint):
        self.client.wait_for_server()

        x,y,deg=waypoint

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        yaw = math.radians(deg)
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


    def state_transition(self):
        self.enemy_policy()
        if(rospy.Time.now().to_sec() - self.pre_time)>6:
            self.state_patrol()
        elif self.enemy_policy_est:
            self.state_track()
        else:
            self.state_sneak()

    
def main():
    rospy.init_node('navirun')
    bot = NaoBot()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        bot.state_transition()
        rate.sleep()
        


if __name__ == '__main__':
    main()