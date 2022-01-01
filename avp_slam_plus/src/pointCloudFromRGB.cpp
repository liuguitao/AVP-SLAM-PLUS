//liuguitao created in 2021.12.22



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
#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <vector>


//  synchronism images of multi cameras according to timestamp
//  generate point cloud according to IPM principle
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

std::queue<sensor_msgs::ImageConstPtr> camera0ImageBuf;
std::queue<sensor_msgs::ImageConstPtr> camera1ImageBuf;
std::queue<sensor_msgs::ImageConstPtr> camera2ImageBuf;
std::queue<sensor_msgs::ImageConstPtr> camera3ImageBuf;
std::queue<sensor_msgs::ImageConstPtr> camera4ImageBuf;
std::queue<sensor_msgs::ImageConstPtr> camera5ImageBuf;


std::mutex mBuf;


double timeCamera0Points=0;
double timeCamera1Points=0;
double timeCamera2Points=0;
double timeCamera3Points=0;
double timeCamera4Points=0;
double timeCamera5Points=0;

double timeCamera0Image=0;
double timeCamera1Image=0;
double timeCamera2Image=0;
double timeCamera3Image=0;
double timeCamera4Image=0;
double timeCamera5Image=0;


cv_bridge::CvImageConstPtr image0_ptr;
cv_bridge::CvImageConstPtr image1_ptr;
cv_bridge::CvImageConstPtr image2_ptr;
cv_bridge::CvImageConstPtr image3_ptr;
cv_bridge::CvImageConstPtr image4_ptr;
cv_bridge::CvImageConstPtr image5_ptr;

Eigen::Matrix3d K0;
Eigen::Matrix3d K1;
Eigen::Matrix3d K2;
Eigen::Matrix3d K3;
Eigen::Matrix3d K4;
Eigen::Matrix3d K5;



Eigen::Matrix3d R0;
Eigen::Vector3d T0;
Eigen::Matrix3d RT0;

Eigen::Matrix3d R1R;
Eigen::Vector3d T1R;
Eigen::Matrix3d R1;
Eigen::Vector3d T1;
Eigen::Matrix3d RT1;


Eigen::Matrix3d R2R;
Eigen::Vector3d T2R;
Eigen::Matrix3d R2;
Eigen::Vector3d T2;
Eigen::Matrix3d RT2;

Eigen::Matrix3d R3R;
Eigen::Vector3d T3R;
Eigen::Matrix3d R3;
Eigen::Vector3d T3;
Eigen::Matrix3d RT3;


Eigen::Matrix3d R4R;
Eigen::Vector3d T4R;
Eigen::Matrix3d R4;
Eigen::Vector3d T4;
Eigen::Matrix3d RT4;


Eigen::Matrix3d R5R;
Eigen::Vector3d T5R;
Eigen::Matrix3d R5;
Eigen::Vector3d T5;
Eigen::Matrix3d RT5;

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
Eigen::Matrix3d K;


int imageRowIncrease=1;
int imageColIncrease=1;
double cameraRealiableDis=15.0;
int skyColor=178;
float pointCloudLeafSize=0.1;
double rotateDeg1,rotateDeg2,rotateDeg3,rotateDeg4,rotateDeg5;

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

void camera0ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera0ImageBuf.push(imageMsg);
    mBuf.unlock();
    

}

void camera1ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera1ImageBuf.push(imageMsg);
    mBuf.unlock();
}

void camera2ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera2ImageBuf.push(imageMsg);
    mBuf.unlock();
}

void camera3ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera3ImageBuf.push(imageMsg);
    mBuf.unlock();
}

void camera4ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera4ImageBuf.push(imageMsg);
    mBuf.unlock();
}

void camera5ImageHandler(const sensor_msgs::ImageConstPtr& imageMsg){

    mBuf.lock();
    camera5ImageBuf.push(imageMsg);
    mBuf.unlock();
}




//   radian to degree
 double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

//   transform degree to quaternion
void degToQuan(double& rotateDeg,Eigen::Quaterniond& q){
    
   geometry_msgs::Quaternion camera0xQuat=tf::createQuaternionMsgFromRollPitchYaw(0,rotateDeg, 0);
   q.w()=camera0xQuat.w;
   q.x()=camera0xQuat.x;
   q.y()=camera0xQuat.y;
   q.z()= camera0xQuat.z;
}


//   compute inner and external parameter of cameras
void systemInit(){
  
    R1R=Eigen::AngleAxisd(-rotateDeg1, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    T1R<<0,0,0;

    R2R=Eigen::AngleAxisd(-rotateDeg2, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    T2R<<0,0,0;


    R3R=Eigen::AngleAxisd(-rotateDeg3, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    T3R<<0,0,0;


    R4R=Eigen::AngleAxisd(-rotateDeg4, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    T4R<<0,0,0;


    R5R=Eigen::AngleAxisd(-rotateDeg5, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    T5R<<0,0,0;


    K0=K;
    RT0<<R0(0,0),R0(0,1),T0(0),
         R0(1,0),R0(1,1),T0(1),
         R0(2,0),R0(2,1),T0(2);
  

    K1=K;
    R1=R1R*R0;
    T1=R1R*T0+T1R;
    RT1<<R1(0,0),R1(0,1),T1(0),
         R1(1,0),R1(1,1),T1(1),
         R1(2,0),R1(2,1),T1(2);

    K2=K;
    R2=R2R*R0;
    T2=R2R*T0+T2R;
    RT2<<R2(0,0),R2(0,1),T2(0),
         R2(1,0),R2(1,1),T2(1),
         R2(2,0),R2(2,1),T2(2);

    K3=K;
    R3=R3R*R0;
    T3=R3R*T0+T3R;
    RT3<<R3(0,0),R3(0,1),T3(0),
         R3(1,0),R3(1,1),T3(1),
         R3(2,0),R3(2,1),T3(2);


    K4=K;
    R4=R4R*R0;
    T4=R4R*T0+T4R;
    RT4<<R4(0,0),R4(0,1),T4(0),
         R4(1,0),R4(1,1),T4(1),
         R4(2,0),R4(2,1),T4(2);

    K5=K;
    R5=R5R*R0;
    T5=R5R*T0+T5R;
    RT5<<R5(0,0),R5(0,1),T5(0),
         R5(1,0),R5(1,1),T5(1),
         R5(2,0),R5(2,1),T5(2);
    
 
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


//   synchronism images of all camera
void removeUnsynData(double& timeMax,double& timeCameraImage,std::queue<sensor_msgs::ImageConstPtr>& cameraImageBuf){
     while((timeCameraImage<timeMax) && !cameraImageBuf.empty()){
                 mBuf.lock();
                 cameraImageBuf.pop();
                 mBuf.unlock();
                 if(!cameraImageBuf.empty())
                     timeCameraImage=cameraImageBuf.front()->header.stamp.toSec();
            }
}

//    compute point cloud according to IPM principle
void  calCloudFromImage(Eigen::Matrix3d& K, Eigen::Matrix3d& RT,const cv::Mat& image,pcl::PointCloud<PointType>::Ptr& cameraCloud){
        Eigen::Matrix3d KInv=K.inverse();
        Eigen::Matrix3d RTInv=RT.inverse();
       int row=image.rows;
       int col=image.cols;


       for(int i=0;i<row;i=i+imageRowIncrease){
           const uchar* p=image.ptr<uchar>(i);
           for(int j=0;j<col;j=j+imageColIncrease){
      
            int b=p[3*j];
            int g=p[3*j+1];
            int r=p[3*j+2];
           //   there,for simplicity,just according to color,detect invalid area like sky area;
           //   for real scene,should adapt machine learning method or other method detecting invalid area
            if(b==skyColor ){
                break;
            }
              
               Eigen::Vector3d u;
               u(0)=j;
               u(1)=i;
               u(2)=1;
               Eigen::Vector3d u1;
          
               u1=KInv*u;
               u1=RTInv*u1;
               u1(0)=u1.x()/u1.z();
               u1(1)=u1.y()/u1.z();
               double dis=sqrt(u1.x()*u1.x()+u1.y()*u1.y());
             
                if(dis>cameraRealiableDis)
                    continue;
               
               PointType po;
               po.x = u1.x();
	           po.y = u1.y();
	           po.z = 0;
               po.r=r;
               po.g=g;
               po.b=b;
               cameraCloud->push_back(po);

           }
       }
}




int main(int argc, char *argv[]){

   ros::init(argc, argv, "test_frameSyn");
   ros::NodeHandle nh;

  // get parameter from config file 
   nh.param<double>("rotateDeg1", rotateDeg1, -1.0471975511965976);
   nh.param<double>("rotateDeg2", rotateDeg2, -2.0943951023931953);
   nh.param<double>("rotateDeg3", rotateDeg3, -3.141592653589793);
   nh.param<double>("rotateDeg4", rotateDeg4, 2.0943951023931953);
   nh.param<double>("rotateDeg5", rotateDeg5, 1.0471975511965976);
   nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
   nh.param<int>("imageRowIncrease", imageRowIncrease, 1);
   nh.param<int>("imageColIncrease", imageColIncrease, 2);
   nh.param<double>("cameraRealiableDis", cameraRealiableDis, 8.0);
   nh.param<int>("skyColor", skyColor, 178);

   
   std::vector<double> KDouble,R0Double,T0Double;
   nh.param<std::vector<double>>("K", KDouble, std::vector<double>());
   nh.param<std::vector<double>>("R0", R0Double, std::vector<double>());
   nh.param<std::vector<double>>("T0", T0Double, std::vector<double>());
   K = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(KDouble.data(), 3, 3);
   R0 = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R0Double.data(), 3, 3);
   T0 = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(T0Double.data(), 3, 1);


   ros::Subscriber subCamera0Image = nh.subscribe("/camera0/color/image_raw", 100, camera0ImageHandler);
   ros::Subscriber subCamera1Image = nh.subscribe("/camera1/color/image_raw", 100, camera1ImageHandler);
   ros::Subscriber subCamera2Image = nh.subscribe("/camera2/color/image_raw", 100, camera2ImageHandler);
   ros::Subscriber subCamera3Image = nh.subscribe("/camera3/color/image_raw", 100, camera3ImageHandler);
   ros::Subscriber subCamera4Image = nh.subscribe("/camera4/color/image_raw", 100, camera4ImageHandler);
   ros::Subscriber subCamera5Image = nh.subscribe("/camera5/color/image_raw", 100, camera5ImageHandler);

   pubCameraCloudFrame = nh.advertise<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100);

   //   compute inner and external parameter of cameras
   systemInit();
   

   ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        //  synchronism images of multi cameras according to timestamp
        if(!camera0ImageBuf.empty() && !camera1ImageBuf.empty() && !camera2ImageBuf.empty() && !camera3ImageBuf.empty() && !camera4ImageBuf.empty() && !camera5ImageBuf.empty()){
            timeCamera0Image=camera0ImageBuf.front()->header.stamp.toSec();
            timeCamera1Image=camera1ImageBuf.front()->header.stamp.toSec();
            timeCamera2Image=camera2ImageBuf.front()->header.stamp.toSec();
            timeCamera3Image=camera3ImageBuf.front()->header.stamp.toSec();
            timeCamera4Image=camera4ImageBuf.front()->header.stamp.toSec();
            timeCamera5Image=camera5ImageBuf.front()->header.stamp.toSec();

            if((timeCamera0Image!=timeCamera1Image) || (timeCamera0Image!=timeCamera2Image) || (timeCamera0Image!=timeCamera3Image) || (timeCamera0Image!=timeCamera4Image) || (timeCamera0Image!=timeCamera5Image)){
                std::cout<<"image is not syn"<<std::endl;

                double timeImageMax=timeCamera0Image;
                timeImageMax=std::max(timeImageMax,timeCamera1Image);
                timeImageMax=std::max(timeImageMax,timeCamera2Image);
                timeImageMax=std::max(timeImageMax,timeCamera3Image);
                timeImageMax=std::max(timeImageMax,timeCamera4Image);
                timeImageMax=std::max(timeImageMax,timeCamera5Image);

                removeUnsynData(timeImageMax,timeCamera0Image,camera0ImageBuf);
                removeUnsynData(timeImageMax,timeCamera1Image,camera1ImageBuf);
                removeUnsynData(timeImageMax,timeCamera2Image,camera2ImageBuf);
                removeUnsynData(timeImageMax,timeCamera3Image,camera3ImageBuf);
                removeUnsynData(timeImageMax,timeCamera4Image,camera4ImageBuf);
                removeUnsynData(timeImageMax,timeCamera5Image,camera5ImageBuf);

                continue;

            }

            
            std::cout<<"image is  syn"<<std::endl;
           

            mBuf.lock();
            image0_ptr = cv_bridge::toCvShare(camera0ImageBuf.front());
            camera0ImageBuf.pop();
            mBuf.unlock();

            mBuf.lock();
            image1_ptr = cv_bridge::toCvShare(camera1ImageBuf.front());
            camera1ImageBuf.pop();
            mBuf.unlock();

             mBuf.lock();
            image2_ptr = cv_bridge::toCvShare(camera2ImageBuf.front());
            camera2ImageBuf.pop();
            mBuf.unlock();

            mBuf.lock();
            image3_ptr = cv_bridge::toCvShare(camera3ImageBuf.front());
            camera3ImageBuf.pop();
            mBuf.unlock();

             mBuf.lock();
            image4_ptr = cv_bridge::toCvShare(camera4ImageBuf.front());
            camera4ImageBuf.pop();
            mBuf.unlock();

            mBuf.lock();
            image5_ptr = cv_bridge::toCvShare(camera5ImageBuf.front());
            camera5ImageBuf.pop();
            mBuf.unlock();

            //for every camera, compute point cloud according to IPM principle
            camera0Cloud->clear();
            calCloudFromImage(K0,RT0,image0_ptr->image,camera0Cloud);

            camera1Cloud->clear();
            calCloudFromImage(K1,RT1,image1_ptr->image,camera1Cloud);

            camera2Cloud->clear();
            calCloudFromImage(K2,RT2,image2_ptr->image,camera2Cloud);

            camera3Cloud->clear();
            calCloudFromImage(K3,RT3,image3_ptr->image,camera3Cloud);

            camera4Cloud->clear();
            calCloudFromImage(K4,RT4,image4_ptr->image,camera4Cloud);

            camera5Cloud->clear();
            calCloudFromImage(K5,RT5,image5_ptr->image,camera5Cloud);

            
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
            cameraFrameCloud->clear();
            *cameraFrameCloud=*cameraFrameCloud+*camera0Cloud;
            *cameraFrameCloud=*cameraFrameCloud+*camera1Cloud;
            *cameraFrameCloud=*cameraFrameCloud+*camera2Cloud;
            *cameraFrameCloud=*cameraFrameCloud+*camera3Cloud;
            *cameraFrameCloud=*cameraFrameCloud+*camera4Cloud;
            *cameraFrameCloud=*cameraFrameCloud+*camera5Cloud;

            // broad point cloud of current frame
            int cameraPointSize=cameraFrameCloud->points.size();
            std::cout<<"point size is"<<cameraPointSize<<std::endl;
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