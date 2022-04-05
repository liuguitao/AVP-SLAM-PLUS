#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import math
import time

#moves = [(0.5,0,120),(0,math.pi/20,20),(0.5,0,60),(0,0,1)]
moves = []

def turnLeft90():
    speed = 0.015
    moves.append((0,-math.pi/2*speed,1/speed))
def turnRight90():
    speed = 0.015
    moves.append((0,math.pi/2*speed,1/speed))
def forward(x):
    speed = 0.1
    moves.append((speed,0,x/speed))
def stop():
    moves.append((0,0,0.1))

distance_from_center = 3
turnLeft90()
forward(distance_from_center)
turnRight90()
forward(3*20)
turnRight90()
forward(distance_from_center*2)
turnRight90()
forward(3*20-distance_from_center)
turnLeft90()
forward(2*20-distance_from_center)
turnRight90()
forward(distance_from_center*2)
turnRight90()
forward(2*20)
stop()

# for i in range(4):
#     forward(5)
#     turnRight90()
# stop()

timeSent = 0
index = 0
twist = Twist()

if __name__=="__main__":

    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.init_node('mrobot_teleop')
    rate = rospy.Rate(100)

    try:

        while not rospy.is_shutdown():

            if(time.time()>timeSent):
                move = moves[index]
                index+=1
                if(index>=len(moves)):
                    break

                print('performing',move,time.time())

                twist.linear.x = move[0]
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = move[1]

                timeSent = time.time()+move[2]*1.0742

            pub.publish(twist)
            rate.sleep()

    except:
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
