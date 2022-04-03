#!/usr/bin/env python
# import string
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

gazebo_c1 = 0.05
gazebo_c2 = 0.05
gazebo_c3 = 0.05
gazebo_c4 = 0.05

odom_c1 = 0.1
odom_c2 = 0.1
odom_c3 = 0.1
odom_c4 = 0.1
dt = 1.0/100
id = 0
pre_id = -1

previous_pose = np.array([0, 0, 0])

def odom_callback(data):
    global vertex_pub
    global edge_pub
    global cmd_pub
    # odom_pub.publish(data)
    # print(data.linear.x)
    # print(data.angular.z)
    v = data.linear.x
    w = data.angular.z
    # print("ideal: ", v, w)

    ######################################################################
    #################### calculate vel cmd for gazebo ####################
    ######################################################################

    gazebo_vel_cmd = Twist()
    g_R = np.array([[(gazebo_c1* abs(v)+gazebo_c2*abs(w))**2, 0], [0, (gazebo_c3* abs(v)+gazebo_c4*abs(w))**2]])
    g_rand = np.random.multivariate_normal([v, w], g_R)
    v_actual = g_rand[0]
    w_actual = g_rand[1]
    # print("actual: ", v_actual, w_actual)
    gazebo_vel_cmd.linear.x = v_actual
    gazebo_vel_cmd.angular.z = w_actual
    cmd_pub.publish(gazebo_vel_cmd)
    ######################################################################
    ################## ready to publish as noisy input ###################
    ######################################################################
    global previous_pose
    global id
    global pre_id
    # odom_vel_cmd = Twist()
    delta_t = dt
    o_R = np.array([[(gazebo_c1* abs(v)+gazebo_c2*abs(w))**2, 0], [0, (gazebo_c3* abs(v)+gazebo_c4*abs(w))**2]])
    o_rand = np.random.multivariate_normal([v, w], o_R)
    v_odom = o_rand[0]
    w_odom = o_rand[1]
    # print("odom: ", v_odom, w_odom)

#     # print("v_actual", v_actual)
#     # print("w_actual", w_actual)
    delta_x = v_odom*delta_t*np.cos(previous_pose[2] + w_odom*delta_t)
    delta_y = v_odom*delta_t*np.sin(previous_pose[2] + w_odom*delta_t)
    delta_theta = w_odom * delta_t
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    new_pose = previous_pose + delta_pose
    # print(new_pose)
    vertex_output = "VERTEX_SE2" + " " +  str(id) + " " + str(new_pose[0]) + " " + str(new_pose[1]) + " " + str(new_pose[2])
    edge_output = "EDGE_SE2" + " " + str(pre_id) + " " + str(id) + " " + str(delta_x) + " " + str(delta_y) + " " + str(delta_theta) + " " + str(o_R[0, 0]) + " " + str(o_R[1, 1])
    print(vertex_output)
    print(edge_output)
    previous_pose = new_pose
    id += 1
    pre_id += 1 
    edge_pub.publish(edge_output)    
    vertex_pub.publish(vertex_output)    

#     return new_pose

if __name__ == '__main__':
    try:
        rospy.init_node('Odometry', anonymous=True)
        vertex_pub = rospy.Publisher('/vertex_odom', String, queue_size=10)
        edge_pub = rospy.Publisher('/edge_odom', String, queue_size=10)
        cmd_pub = rospy.Publisher('/actual_cmd_vel', Twist, queue_size=10)

        # subscribe ctrl_cmd
        rospy.Subscriber("/cmd_vel", Twist, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass