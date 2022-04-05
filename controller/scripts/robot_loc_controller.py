#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_Angle = rospy.Publisher('/Angle', Twist, queue_size=10)
# pub_GoalAngle = rospy.Publisher('/GoalAngle', Twist, queue_size=10)
# initialize pose
x = 0.0
y = 0.0
yaw_angle = 0.0
rho = 10.0
Ang_Dif = 10.0
# parameters
# goal_x = [1.0, 0.0, 1.0, 0.0]
# goal_y = [0.0, 1.0, 0.0, 1.0]
goal_x = [0.5, 3.5, 3.5, 0.5]
goal_x = goal_x + goal_x + goal_x
goal_x.append(0.5)
goal_y = [-1.0, -1.0, 2.0, 2.0]
goal_y = goal_y + goal_y + goal_y
goal_y.append(-1.0)
goal_theta = [0, 180, 0, -180]
k_rho = 0.2
k_alpha = 1.5 #1.0
k_beta = -0.3
Dist_tolerance = 0.05
Ang_tolerance = 1.0

Stop = Twist()
Stop.linear.x = 0
Stop.angular.z = 0
VelLimit = 0.2
phase_flag = 0

# def Shortest_Ang():

def Pose_callback(Odometry):
    global x
    global y
    x = Odometry.pose.pose.position.x
    y = Odometry.pose.pose.position.y
    # print("Now is at: ", x, y)

def imu_callback(orientation):
    global yaw_angle
    yaw_angle = orientation.pose.position.z
    # print(yaw_angle)

def vel_limit(vel_x):
    if vel_x > VelLimit:
        # print("Limited")
        return VelLimit
    else:
        return vel_x

def control_command(point_index):
    delta_x = (goal_x[point_index] - x)
    delta_y = (goal_y[point_index] - y)
    theta = yaw_angle
    global Ang_Dif
    # Ang_Dif = goal_theta[point_index] - theta
    global rho
    rho = math.sqrt(delta_x**2 + delta_y**2)
    goal_angle = math.atan2(delta_y, delta_x)*180/math.pi
    alpha = -theta + goal_angle

    Angle = Twist()
    Angle.linear.x = goal_angle
    Angle.linear.y = alpha
    Angle.linear.z = rho
    global pub_Angle
    pub_Angle.publish(Angle)

    # beta = goal_theta[point_index] -theta - alpha
    if abs(alpha) > abs(alpha + 360):
        alpha = alpha + 360
    elif abs(alpha - 360) < abs(alpha):
        alpha = alpha - 360
    else:
        pass
    print("X: %f, Y: %f, Yaw Angle: %f" %(x, y, theta))

    # print("atan2(delta_y, delta_x)", math.atan2(delta_y, delta_x)*180/math.pi)
    # print("theta", theta)
    # print("alpha", alpha)
    # Alpha = alpha
    alpha = alpha * math.pi/180
    # beta = beta * math.pi/180
    global phase_flag
    vel = Twist()
    # if abs(Alpha) >= 2 and phase_flag == 0:
    #     print("Phase 0")
    #     vel.linear.x = 0
    #     vel.angular.z = k_alpha * alpha
    # else:
    #     print("Phase 1")
    #     phase_flag = 1
    vel_x = k_rho * rho
    vel_x = vel_limit(vel_x)
    vel_x = math.cos(alpha) * vel_x
    vel.linear.x = vel_x
    vel.angular.z = k_alpha * alpha

    global pub
    pub.publish(vel)

def ReachedGoal():
    rospy.loginfo("Reached Goal!!!")
    global pub
    pub.publish(Stop)


def start():
    rospy.Subscriber("/odom_filtered_map", Odometry, Pose_callback)
    rospy.Subscriber("/yaw_filtered_map", PoseStamped, imu_callback)
    
    rospy.init_node('robot_loc_controller')
    rate = rospy.Rate(100) # 10hz
    point_index = 0
    goal_flag = 0
    global phase_flag
    while not rospy.is_shutdown():
        # print(goal_x)
        if goal_flag == 1:
            ReachedGoal()
            break
            # rospy.signal_shutdown("Reached Goal!!!")
        # elif rho <= Dist_tolerance and Ang_Dif <= Ang_tolerance:
        elif rho <= Dist_tolerance and goal_flag == 0:
            point_index += 1
            phase_flag = 0
            if point_index == len(goal_x):
                goal_flag = 1
                continue
            control_command(point_index)
        else:
            control_command(point_index)
        rate.sleep()

if __name__ == '__main__':
    start()