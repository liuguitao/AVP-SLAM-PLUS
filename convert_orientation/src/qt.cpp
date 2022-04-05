#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include <nav_msgs/Odometry.h>

int first_flag = 0;
float ini_yaw = 0;
ros::Publisher yaw_publisher;
ros::Subscriber quat_subscriber;
ros::Publisher odom_yaw_publisher;
ros::Subscriber odom_quat_subscriber;
void QuaternionCallback(const sensor_msgs::Imu msg)
{
    double roll, pitch, yaw;
    //Transform quaternion to rotation
    tf::Quaternion quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    
    if(first_flag == 0)
    {
        ini_yaw = yaw*180/M_PI;
        first_flag = 1;
    }
    //Publiish the rotation values
    geometry_msgs::PoseStamped rpy;
    rpy.header = msg.header;
    rpy.pose.position.x = roll;
    rpy.pose.position.y = pitch;
    //rpy.pose.position.z = yaw; //yaw is shown as in rad
    rpy.pose.position.z = yaw*180/M_PI - ini_yaw; //yaw is shown as in degrees
    yaw_publisher.publish(rpy);
    ROS_INFO("published yaw angle(degrees): yaw=%f", rpy.pose.position.z);
}

void OdomQuaternionCallback(const nav_msgs::Odometry odom_msg)
{
    double roll, pitch, yaw;
    //Transform quaternion to rotation
    tf::Quaternion quat(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //Publiish the rotation values
    geometry_msgs::PoseStamped rpy;
    rpy.header = odom_msg.header;
    rpy.pose.position.x = roll;
    rpy.pose.position.y = pitch;
    //rpy.pose.position.z = yaw; //yaw is shown as in rad
    rpy.pose.position.z = yaw*180/M_PI; //yaw is shown as in degrees
    odom_yaw_publisher.publish(rpy);
    // ROS_INFO("published odom yaw angle(degrees): yaw=%f", rpy.pose.position.z*180/M_PI);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion");
    ros::NodeHandle n;
    
    // Publisher and Subscriber (Subscribe IMU)
    yaw_publisher = n.advertise<geometry_msgs::PoseStamped>("yaw_imu", 1000);
    quat_subscriber = n.subscribe("/imu", 1000, QuaternionCallback);
    odom_yaw_publisher = n.advertise<geometry_msgs::PoseStamped>("yaw_odom", 1000);
    odom_quat_subscriber = n.subscribe("/odom", 1000, OdomQuaternionCallback);
    //Check for incoming quaternions
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
