#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

Stop = Twist()
Stop.linear.x = 0
Stop.angular.z = 0
VelLimit = 0.2 # 5s -> 1m
AngVelLimit = math.pi/6 # 3s -> pi/2

square_length = 1.5
square_width = 1

def vel_limit(vel_x):
    if vel_x > VelLimit:
        # print("Limited")
        return VelLimit
    else:
        return vel_x

# def control_command(point_index):
#     t0 = rospy.Time.now().to_sec()
#     print(t0)

def MoveStraight(length, direction):
    travel_time = length/VelLimit
    vel = Twist()
    stop_flag = 0
    t0_flag = 0
    while (t0_flag == 0):
        if rospy.Time.now().to_sec() != 0:
            t0 = rospy.Time.now().to_sec()
            t0_flag =1
    vel.linear.x = VelLimit * direction
    global pub
    print("publish cmd_vel")
    while(stop_flag == 0):
        pub.publish(vel)
        # print(t0)
        # print("waiting")
        t1 = rospy.Time.now().to_sec()
        duration = t1 - t0
        print(duration)
        if (duration) >= travel_time:
            stop_flag = 1
            print("stop")
            pub.publish(Stop)

def Turn(turning_angle):
    travel_time = abs(turning_angle)/AngVelLimit
    if turning_angle > 0:
        direction = 1
    else:
        direction = -1 
    vel = Twist()
    stop_flag = 0
    t0_flag = 0
    while (t0_flag == 0):
        if rospy.Time.now().to_sec() != 0:
            t0 = rospy.Time.now().to_sec()
            t0_flag =1
    vel.angular.z = AngVelLimit * direction
    global pub
    print("publish cmd_vel")
    while(stop_flag == 0):
        pub.publish(vel)
        # print(t0)
        # print("waiting")
        t1 = rospy.Time.now().to_sec()
        duration = t1 - t0
        print(duration)
        if (duration) >= travel_time:
            stop_flag = 1
            print("stop")
            pub.publish(Stop)

def start():
    # rospy.Subscriber("/odom_filtered_map", Odometry, Pose_callback)
    # rospy.Subscriber("/yaw_filtered_map", PoseStamped, imu_callback)
    
    rospy.init_node('new_controller')
    rate = rospy.Rate(100) # 10hz
    # point_index = 0
    # goal_flag = 0
    # global phase_flag
    while not rospy.is_shutdown():
        # print(goal_x)
        # if goal_flag == 1:
        #     ReachedGoal()
        #     break
            # rospy.signal_shutdown("Reached Goal!!!")
        # elif rho <= Dist_tolerance and Ang_Dif <= Ang_tolerance:
        # elif rho <= Dist_tolerance and goal_flag == 0:
        #     point_index += 1
        #     phase_flag = 0
        #     if point_index == len(goal_x):
        #         goal_flag = 1
        #         continue
        #     control_command(point_index)
        # else:
        #     control_command(point_index)
        # t0 = rospy.Time.now().to_sec()
        # t1 = rospy.Time.now().to_sec()
        # print(t0)
        MoveStraight(square_length, 1)
        Turn(math.pi/2)
        MoveStraight(square_width, 1)
        rate.sleep()
        break

if __name__ == '__main__':
    start()