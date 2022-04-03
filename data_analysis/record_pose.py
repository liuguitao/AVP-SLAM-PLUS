#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

lastGT = Pose()
GTList = []
poseList = []

GT = rospy.Publisher('/GT', Odometry, queue_size=10)

def GroundTruthCallback(data):
    global lastGT
    pose = data.pose[6]
    pose.position.y*=-1
    lastGT = pose
    #GT Y is flipped to match the RVIZ viewer

    #rospy.loginfo("GT %f %f %f", pose.position.x, -pose.position.y, pose.position.z)

def PoseCallback(data):
    global lastGT
    pose = data.pose.pose
    poseList.append([pose.position.x,pose.position.y,pose.position.z])
    GTList.append([lastGT.position.x,lastGT.position.y,lastGT.position.z])
    
    rospy.loginfo("Pose %f %f %f", pose.position.x, pose.position.y, pose.position.z)
    rospy.loginfo("GT %f %f %f", lastGT.position.x, lastGT.position.y, lastGT.position.z)

    data.pose.pose = lastGT
    GT.publish(data)

if __name__=="__main__":

    rospy.init_node('data_listener')
    rospy.Subscriber("/gazebo/link_states", LinkStates, GroundTruthCallback)
    rospy.Subscriber("/currentPose", Odometry, PoseCallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    GTList = np.array(GTList)
    poseList = np.array(poseList)
    print(GTList.shape,poseList.shape)

    plt.subplot(311)

    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(GTList[:,0],label='GT')
    ax1.plot(poseList[:,0],label='pose')
    ax1.set(ylabel='x')
    ax1.legend()
    
    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(GTList[:,1])
    ax2.plot(poseList[:,1])
    ax2.set(ylabel='y')
    
    ax3 = plt.subplot(3, 1, 3)
    ax3.plot(GTList[:,2])
    ax3.plot(poseList[:,2])
    ax3.set(ylabel='z')
    
    plt.show()

    np.save('GTList.npy',GTList)
    np.save('poseList.npy',poseList)


