#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include <nav_msgs/Odometry.h>

ros::Publisher map_yaw_publisher;
ros::Subscriber map_quat_subscriber;
ros::Publisher odom_yaw_publisher;
ros::Subscriber odom_quat_subscriber;

void MapQuaternionCallback(const nav_msgs::Odometry map_msg)
{
    double roll, pitch, yaw;
    //Transform quaternion to rotation
    tf::Quaternion quat(map_msg.pose.pose.orientation.x, map_msg.pose.pose.orientation.y, map_msg.pose.pose.orientation.z, map_msg.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //Publiish the rotation values
    geometry_msgs::PoseStamped rpy;
    rpy.header = map_msg.header;
    rpy.pose.position.x = roll;
    rpy.pose.position.y = pitch;
    //rpy.pose.position.z = yaw; //yaw is shown as in rad
    rpy.pose.position.z = yaw*180/M_PI; //yaw is shown as in degrees
    map_yaw_publisher.publish(rpy);
    ROS_INFO("published map yaw angle(degrees): yaw=%f", rpy.pose.position.z);
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
    ROS_INFO("published odom yaw angle(degrees): yaw=%f", rpy.pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion");
    ros::NodeHandle n;
    
    // Publisher and Subscriber (Subscribe IMU)
    map_yaw_publisher = n.advertise<geometry_msgs::PoseStamped>("yaw_filtered_map", 1000);
    map_quat_subscriber = n.subscribe("odom_filtered_map", 1000, MapQuaternionCallback);
    odom_yaw_publisher = n.advertise<geometry_msgs::PoseStamped>("yaw_filtered_odom", 1000);
    odom_quat_subscriber = n.subscribe("odom_filtered_odom", 1000, OdomQuaternionCallback);
    //Check for incoming quaternions
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
