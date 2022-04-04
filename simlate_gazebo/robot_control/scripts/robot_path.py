#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import math
import time

#moves = [(0.5,0,120),(0,math.pi/20,20),(0.5,0,60),(0,0,1)]
moves = []

def turnLeft90():
    moves.append((0,-math.pi/20,10))
def turnRight90():
    moves.append((0,math.pi/20,10))
def forward(x):
    speed = 0.4
    moves.append((speed,0,x/speed))
def stop():
    moves.append((0,0,0.1))

distance_from_center = 3
# turnLeft90()
# forward(distance_from_center)
# turnRight90()
# forward(3*20)
# turnRight90()
# forward(distance_from_center*2)
# turnRight90()
# forward(3*20-distance_from_center)
# turnLeft90()
# forward(2*20-distance_from_center)
# turnRight90()
# forward(distance_from_center*2)
# turnRight90()
# forward(2*20)

# turnRight90()
forward(15)
turnRight90()
forward(5)
turnRight90()
forward(15)
turnRight90()
forward(5)
turnRight90()
forward(5)
turnRight90()
forward(15)
turnRight90()
forward(5)
turnRight90()
forward(15)
# turnRight90()
stop()

timeSent = 0
index = 0
twist = Twist()

if __name__=="__main__":
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # pub = rospy.Publisher('/ideal_cmd_vel', Twist, queue_size=5)
    rospy.init_node('mrobot_teleop')
    rate = rospy.Rate(100)

    # # wait for other node to start
    # rospy.sleep(1)
    while not rospy.is_shutdown():

        if(time.time()>timeSent):
            move = moves[index]
            index+=1
            if(index>len(moves)):
                exit()

            print('performing',move,time.time())

            twist.linear.x = move[0]; 
            twist.linear.y = 0; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = move[1]

            timeSent = time.time()+move[2]*1.40
            print("1.40")

        pub.publish(twist)
        rate.sleep()
