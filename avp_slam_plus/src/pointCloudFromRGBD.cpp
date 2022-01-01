//liuguitao created in 2021.12.14

#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <queue>
#include <cmath>




//  synchronism point cloud of multi cameras according to timestamp
//  generate point cloud of current frame according to external parameter
//  broadcast point cloud of current frame


ros::Publisher pubCamera0CloudNew;
ros::Publisher pubCamera1CloudNew;
ros::Publisher pubCameraCloudFrame;

typedef pcl::PointXYZRGB PointType;
pcl::PointCloud<PointType>::Ptr camera0Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr camera1Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr camera2Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr camera3Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr camera4Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr camera5Cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cameraFrameCloud(new pcl::PointCloud<PointType>());
std::queue<sensor_msgs::PointCloud2ConstPtr> camera0CloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> camera1CloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> camera2CloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> camera3CloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> camera4CloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> camera5CloudBuf;

std::mutex mBuf;


double timeCamera0Points=0;
double timeCamera1Points=0;
double timeCamera2Points=0;
double timeCamera3Points=0;
double timeCamera4Points=0;
double timeCamera5Points=0;

float closePointThresh=0.1;
float farPointThresh=20;
float pointCloudLeafSize=0.1;


// delete close point
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


// delete far point
template <typename PointT>
void removeFarPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z > thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void camera0CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera0CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();
}


void camera1CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera1CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();

}
void camera2CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera2CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();
}
void camera3CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera3CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();
}
void camera4CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera4CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();
}
void camera5CloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    mBuf.lock();
    camera5CloudBuf.push(cameraCloudMsg);
    mBuf.unlock();
}


//   radian to degree
 double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



 Eigen::Quaterniond q_c0c1(1,0,0,0);
 Eigen::Vector3d t_c0c1(0, 0, 0);

 Eigen::Quaterniond q_c0c2(1,0,0,0);
 Eigen::Vector3d t_c0c2(0, 0, 0);

 Eigen::Quaterniond q_c0c3(1,0,0,0);
 Eigen::Vector3d t_c0c3(0, 0, 0);

 Eigen::Quaterniond q_c0c4(1,0,0,0);
 Eigen::Vector3d t_c0c4(0, 0, 0);

 Eigen::Quaterniond q_c0c5(1,0,0,0);
 Eigen::Vector3d t_c0c5(0, 0, 0);

//   transform degree to quaternion
void degToQuan(double& rotateDeg,Eigen::Quaterniond& q){
    
   geometry_msgs::Quaternion camera0xQuat=tf::createQuaternionMsgFromRollPitchYaw(0,rotateDeg, 0);
   q.w()=camera0xQuat.w;
   q.x()=camera0xQuat.x;
   q.y()=camera0xQuat.y;
   q.z()= camera0xQuat.z;
}



void transformToFrame(PointType const *const pi, PointType *const po, Eigen::Quaterniond q, Eigen::Vector3d t)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q * point_curr + t;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
    po->r=pi->r;
    po->g=pi->g;
    po->b=pi->b;
	
}

//   synchronism point cloud of all camera
void removeUnsynData(double& timeMax,double& timeCameraPoints,std::queue<sensor_msgs::PointCloud2ConstPtr>& cameraCloudBuf){
     while((timeCameraPoints<timeMax) && !cameraCloudBuf.empty()){
                 mBuf.lock();
                 cameraCloudBuf.pop();
                 mBuf.unlock();
                 if(!cameraCloudBuf.empty())
                     timeCameraPoints=cameraCloudBuf.front()->header.stamp.toSec();
            }
}




int main(int argc, char *argv[]){

   ros::init(argc, argv, "test_frameSyn");
   ros::NodeHandle nh;

   // get parameter from config file 
   double rotateDeg1,rotateDeg2,rotateDeg3,rotateDeg4,rotateDeg5;
   nh.param<double>("rotateDeg1", rotateDeg1, -1.0471975511965976);
   nh.param<double>("rotateDeg2", rotateDeg2, -2.0943951023931953);
   nh.param<double>("rotateDeg3", rotateDeg3, -3.141592653589793);
   nh.param<double>("rotateDeg4", rotateDeg4, 2.0943951023931953);
   nh.param<double>("rotateDeg5", rotateDeg5, 1.0471975511965976);
   nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
   nh.param<float>("farPointThresh", farPointThresh, 20);
   nh.param<float>("closePointThresh", closePointThresh, 0.1);
 
   degToQuan(rotateDeg1,q_c0c1);
   degToQuan(rotateDeg2,q_c0c2);
   degToQuan(rotateDeg3,q_c0c3);
   degToQuan(rotateDeg4,q_c0c4);
   degToQuan(rotateDeg5,q_c0c5);

   ros::Subscriber subcamera0Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera0/depth/color/points", 100, camera0CloudHandler);
   ros::Subscriber subcamera1Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera1/depth/color/points", 100, camera1CloudHandler);
   ros::Subscriber subcamera2Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera2/depth/color/points", 100, camera2CloudHandler);
   ros::Subscriber subcamera3Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera3/depth/color/points", 100, camera3CloudHandler);
   ros::Subscriber subcamera4Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera4/depth/color/points", 100, camera4CloudHandler);
   ros::Subscriber subcamera5Cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera5/depth/color/points", 100, camera5CloudHandler);

   
   pubCameraCloudFrame = nh.advertise<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100);
   ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        //  synchronism point cloud of multi cameras according to timestamp
        if(!camera0CloudBuf.empty() && !camera1CloudBuf.empty() && !camera2CloudBuf.empty() && !camera3CloudBuf.empty() && !camera4CloudBuf.empty() && !camera5CloudBuf.empty()){
            
            timeCamera0Points=camera0CloudBuf.front()->header.stamp.toSec();
            timeCamera1Points=camera1CloudBuf.front()->header.stamp.toSec();
            timeCamera2Points=camera2CloudBuf.front()->header.stamp.toSec();
            timeCamera3Points=camera3CloudBuf.front()->header.stamp.toSec();
            timeCamera4Points=camera4CloudBuf.front()->header.stamp.toSec();
            timeCamera5Points=camera5CloudBuf.front()->header.stamp.toSec();

         
            if((timeCamera0Points != timeCamera1Points)  || (timeCamera0Points != timeCamera2Points) || (timeCamera0Points != timeCamera3Points) || (timeCamera0Points != timeCamera4Points) || (timeCamera0Points != timeCamera5Points)  ){
                std::cout<<"6 camera now  is not syn  "<<std::endl;

                double timeMax=timeCamera0Points;
                timeMax=std::max(timeMax,timeCamera1Points);
                timeMax=std::max(timeMax,timeCamera2Points);
                timeMax=std::max(timeMax,timeCamera3Points);
                timeMax=std::max(timeMax,timeCamera4Points);
                timeMax=std::max(timeMax,timeCamera5Points);

                removeUnsynData(timeMax,timeCamera0Points,camera0CloudBuf);
                removeUnsynData(timeMax,timeCamera1Points,camera1CloudBuf);
                removeUnsynData(timeMax,timeCamera2Points,camera2CloudBuf);
                removeUnsynData(timeMax,timeCamera3Points,camera3CloudBuf);
                removeUnsynData(timeMax,timeCamera4Points,camera4CloudBuf);
                removeUnsynData(timeMax,timeCamera5Points,camera5CloudBuf);

                continue;
            }
            else{
                std::cout<<"6 camera now is  syn  "<<std::endl;
            }

            mBuf.lock();
            camera0Cloud->clear();
            pcl::fromROSMsg(*camera0CloudBuf.front(), *camera0Cloud);
            camera0CloudBuf.pop();

             camera1Cloud->clear();
             pcl::fromROSMsg(*camera1CloudBuf.front(), *camera1Cloud);
             camera1CloudBuf.pop();
              
              
            camera2Cloud->clear();
            pcl::fromROSMsg(*camera2CloudBuf.front(), *camera2Cloud);
            camera2CloudBuf.pop();

            
            camera3Cloud->clear();
            pcl::fromROSMsg(*camera3CloudBuf.front(), *camera3Cloud);
            camera3CloudBuf.pop();

             
            camera4Cloud->clear();
            pcl::fromROSMsg(*camera4CloudBuf.front(), *camera4Cloud);
            camera4CloudBuf.pop();

           
            camera5Cloud->clear();
            pcl::fromROSMsg(*camera5CloudBuf.front(), *camera5Cloud);
            camera5CloudBuf.pop();

            cameraFrameCloud->clear();
 
            mBuf.unlock();

            // filter point cloud, decrease number of point cloud 
            pcl::PointCloud<PointType> camera0CloudDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(camera0Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(camera0CloudDS);
            *camera0Cloud=camera0CloudDS;

            pcl::PointCloud<PointType> camera1CloudDS;
            downSizeFilter.setInputCloud(camera1Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(camera1CloudDS);
            *camera1Cloud=camera1CloudDS;

            pcl::PointCloud<PointType> camera2CloudDS;
            downSizeFilter.setInputCloud(camera2Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(camera2CloudDS);
            *camera2Cloud=camera2CloudDS;

            pcl::PointCloud<PointType> camera3CloudDS;
            downSizeFilter.setInputCloud(camera3Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize,pointCloudLeafSize);
            downSizeFilter.filter(camera3CloudDS);
            *camera3Cloud=camera3CloudDS;

            pcl::PointCloud<PointType> camera4CloudDS;
            downSizeFilter.setInputCloud(camera4Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(camera4CloudDS);
            *camera4Cloud=camera4CloudDS;

            pcl::PointCloud<PointType> camera5CloudDS;
            downSizeFilter.setInputCloud(camera5Cloud);
            downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
            downSizeFilter.filter(camera5CloudDS);
            *camera5Cloud=camera5CloudDS;

            // accumulate point cloud of every camera 
            int camera1PointSize=camera1Cloud->points.size();
            int camera2PointSize=camera2Cloud->points.size();
            int camera3PointSize=camera3Cloud->points.size();
            int camera4PointSize=camera4Cloud->points.size();
            int camera5PointSize=camera5Cloud->points.size();

            for(int i=0;i<camera1PointSize;i++){
               transformToFrame(&camera1Cloud->points[i],&camera1Cloud->points[i],q_c0c1,t_c0c1);
            }

             for(int i=0;i<camera2PointSize;i++){
               transformToFrame(&camera2Cloud->points[i],&camera2Cloud->points[i],q_c0c2,t_c0c2);
            }

             for(int i=0;i<camera3PointSize;i++){
               transformToFrame(&camera3Cloud->points[i],&camera3Cloud->points[i],q_c0c3,t_c0c3);
            }

             for(int i=0;i<camera4PointSize;i++){
               transformToFrame(&camera4Cloud->points[i],&camera4Cloud->points[i],q_c0c4,t_c0c4);
            }

             for(int i=0;i<camera5PointSize;i++){
               transformToFrame(&camera5Cloud->points[i],&camera5Cloud->points[i],q_c0c5,t_c0c5);
            }

          


             *cameraFrameCloud=*cameraFrameCloud+*camera0Cloud;
             *cameraFrameCloud=*cameraFrameCloud+*camera1Cloud;
             *cameraFrameCloud=*cameraFrameCloud+*camera2Cloud;
             *cameraFrameCloud=*cameraFrameCloud+*camera3Cloud;
             *cameraFrameCloud=*cameraFrameCloud+*camera4Cloud;
             *cameraFrameCloud=*cameraFrameCloud+*camera5Cloud;
            

              //  remove far or close point
             std::vector<int> indices;
             removeClosedPointCloud(*cameraFrameCloud, *cameraFrameCloud, closePointThresh);
             removeFarPointCloud(*cameraFrameCloud, *cameraFrameCloud, farPointThresh);

            sensor_msgs::PointCloud2 cameraCloudFrameMsg;
            pcl::toROSMsg(*cameraFrameCloud, cameraCloudFrameMsg);
            cameraCloudFrameMsg.header.stamp = ros::Time::now();
            cameraCloudFrameMsg.header.frame_id = "/camera0_link";
            pubCameraCloudFrame.publish(cameraCloudFrameMsg);
     
        }

        rate.sleep();
    }


    std::cout<<"hello slam"<<std::endl;
    return 0;
}