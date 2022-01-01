//liuguitao created in 2021.12.16



#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <queue>
#include <cmath>



//   receive point cloud ,then extract feature; 
//   match current feature point cloud to  global map;
//   compute robot's pose with method of ndt or icp; 
//   create new keyFrame, add pointCloud to global map


typedef pcl::PointXYZRGB PointType;

ros::Publisher pubCurrentFeature;
ros::Publisher pubCurrentFeatureInWorld;

ros::Publisher pubGlobalFeature;
ros::Publisher pubCurrentPose;

pcl::PointCloud<PointType>::Ptr currentFeatureCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr globalFeatureCloud(new pcl::PointCloud<PointType>());

bool systemInitial=false;

double odomKeyFramDisThresh=1.0;

double lastPoseX=0;
double lastPoseY=0;
double lastPoseZ=0;
double lastPoseRoll=0;
double lastPosePitch=0;
double lastPoseYaw=0;

float currentX=0;
float currentY=0;
float currentZ=0;
float currentRoll=0;
float currentPitch=0;
float currentYaw=0;

int frameCount=0;
int invalidColorThresh=60;

double icpMaxCorrespondenceDistance=20;
int icpMaximumIterations=100;
double icpTransformationEpsilon=1e-10;
double icpEuclideanFitnessEpsilon=0.001;
double icpFitnessScoreThresh=0.3;

double ndtTransformationEpsilon=1e-10;
double ndtResolution=0.1;
double ndtFitnessScoreThresh=0.3;

std::string mapSaveLocation="/home/lgt/multicamera_ws/src/avp_slam/data/icpLaserCloud.pcd";
float pointCloudLeafSize=0.1;
bool useICP=true;
bool useNDT=false; 
bool mapSave=true;

Eigen::Affine3f transWorldCurrent;

//   transform point cloud according to pose
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Affine3f& transCur)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].r = pointFrom.r;
            cloudOut->points[i].g = pointFrom.g;
            cloudOut->points[i].b = pointFrom.b;

        }
        return cloudOut;
    }





//   feature extration
//   feature registration
//   compute robot's pose
//   create new keyFrame, and add pointCloud to global map

void cameraCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cameraCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cameraCloudIn;
    pcl::fromROSMsg(*cameraCloudMsg, cameraCloudIn);
    
    
    //   extract feature ;
    //   there,for simplicity,just according to color;
    //   for real scene,should adapt deep learning method,detecting valid feature point
    currentFeatureCloud->clear();
    for(size_t i=0;i<cameraCloudIn.points.size();i++){
         PointType pi=cameraCloudIn.points[i];
         int r=int(pi.r);
         int g=int(pi.g);
         int b=int(pi.b);
        if(r<invalidColorThresh && b<invalidColorThresh && g<invalidColorThresh){
            continue;
        }
        currentFeatureCloud->push_back(pi);
    }
 
   
    //  broadcast  feature point cloud of current frame
    sensor_msgs::PointCloud2 cameraCloudFrameMsg;
    pcl::toROSMsg(*currentFeatureCloud, cameraCloudFrameMsg);
    cameraCloudFrameMsg.header.stamp = ros::Time::now();
    cameraCloudFrameMsg.header.frame_id = "/camera0_link";
    pubCurrentFeature.publish(cameraCloudFrameMsg);

     // only number of point of current frame is sufficient ,compute pose 
    if(currentFeatureCloud->points.size()<10)
       return ;

    if(!systemInitial){
        
        //  global map initialize
        *globalFeatureCloud=*globalFeatureCloud+*currentFeatureCloud;
        
        lastPoseX=0;
        lastPoseY=0;
        lastPoseZ=0;
        systemInitial=true;
        return ;
    }
    
    // compute robot's pose with method of ndt
    if(useNDT){
       pcl::NormalDistributionsTransform<PointType, PointType> ndt;
       ndt.setTransformationEpsilon(ndtTransformationEpsilon);
       ndt.setResolution(ndtResolution);
       ndt.setInputSource(currentFeatureCloud);
       ndt.setInputTarget(globalFeatureCloud);
       pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
       ndt.align(*transCurrentCloudInWorld, transWorldCurrent.matrix());
       if (ndt.hasConverged() == false || ndt.getFitnessScore() > ndtFitnessScoreThresh) {
               std::cout << "ndt locolization failed    the score is   " << ndt.getFitnessScore() << std::endl;
                return ;
        } 
        else 
                transWorldCurrent =ndt.getFinalTransformation();
    }
     
    //   compute robot's pose with method of icp   
    if(useICP){
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance); 
        icp.setMaximumIterations(icpMaximumIterations);
        icp.setTransformationEpsilon(icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(icpEuclideanFitnessEpsilon);

        icp.setInputSource(currentFeatureCloud);
        icp.setInputTarget(globalFeatureCloud);
        pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
        icp.align(*transCurrentCloudInWorld,transWorldCurrent.matrix());
            

        if (icp.hasConverged() == false || icp.getFitnessScore() > icpFitnessScoreThresh) {
                std::cout << "ICP locolization failed    the score is   " << icp.getFitnessScore() << std::endl;
                return ;
        } 
        else 
            transWorldCurrent = icp.getFinalTransformation();
    }


    pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);

    


    tf::TransformBroadcaster tfMap2Camera;
    tf::Transform mapToCamera = tf::Transform(tf::createQuaternionFromRPY(currentRoll,currentPitch,currentYaw), tf::Vector3(currentX,currentY,currentZ));
    tfMap2Camera.sendTransform(tf::StampedTransform(mapToCamera, ros::Time::now(), "/map", "/camera0_link"));


    // broacast pose of robot
    geometry_msgs::Quaternion cameraPoseQuat=tf::createQuaternionMsgFromRollPitchYaw(currentRoll,currentPitch,currentYaw);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/map";
    odomAftMapped.child_frame_id = "/camera0_link";
    odomAftMapped.header.stamp = ros::Time::now();
    odomAftMapped.pose.pose.orientation.x = cameraPoseQuat.x;
    odomAftMapped.pose.pose.orientation.y = cameraPoseQuat.y;
    odomAftMapped.pose.pose.orientation.z = cameraPoseQuat.z;
    odomAftMapped.pose.pose.orientation.w = cameraPoseQuat.w;
    odomAftMapped.pose.pose.position.x = currentX;
    odomAftMapped.pose.pose.position.y = currentY;
    odomAftMapped.pose.pose.position.z = currentZ;
    pubCurrentPose.publish(odomAftMapped);

    //   transform  point cloud from current  coordinate to world coordinate
    std::cout<<"now robot is in x "<<transWorldCurrent(0,3)<< " y "<< transWorldCurrent(1,3)<< "  z  "<<transWorldCurrent(2,3)<<std::endl;
    *currentFeatureCloudInWorld =*transformPointCloud(currentFeatureCloud, transWorldCurrent);
    sensor_msgs::PointCloud2 cameraCloudCurrentInWorldMsg;
    pcl::toROSMsg(*currentFeatureCloudInWorld, cameraCloudCurrentInWorldMsg);
    cameraCloudCurrentInWorldMsg.header.stamp = ros::Time::now();
    cameraCloudCurrentInWorldMsg.header.frame_id = "/map";
    pubCurrentFeatureInWorld.publish(cameraCloudCurrentInWorldMsg);

    //  create new keyFrame, and add pointCloud to global map
    double dis=(currentX-lastPoseX)*(currentX-lastPoseX)+(currentY-lastPoseY)*(currentY-lastPoseY)+(currentZ-lastPoseZ)*(currentZ-lastPoseZ);
    frameCount++;
    if(dis>odomKeyFramDisThresh){
        *globalFeatureCloud = *globalFeatureCloud+*currentFeatureCloudInWorld;
        pcl::PointCloud<PointType> globalMapDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(globalFeatureCloud);
        downSizeFilter.setLeafSize(pointCloudLeafSize, pointCloudLeafSize, pointCloudLeafSize);
        downSizeFilter.filter(globalMapDS);
        *globalFeatureCloud=globalMapDS;

        frameCount=0;
        lastPoseX=currentX;
        lastPoseY=currentY;
        lastPoseZ=currentZ;


    }
    //  broadcast prior  global map information 
    sensor_msgs::PointCloud2 cameraCloudGlobalMapMsg;
    pcl::toROSMsg(*globalFeatureCloud, cameraCloudGlobalMapMsg);
    cameraCloudGlobalMapMsg.header.stamp = ros::Time::now();
    cameraCloudGlobalMapMsg.header.frame_id = "/map";
    pubGlobalFeature.publish(cameraCloudGlobalMapMsg);


}


//  save map for relocation
void saveMap_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::cout << "map save start" << std::endl;
    pcl::PointCloud<PointType>  globalMap=*globalFeatureCloud;
    pcl::io::savePCDFileASCII(mapSaveLocation, *globalFeatureCloud);        
	std::cout << "map save over" << std::endl;
   

}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "test_odom");
    ros::NodeHandle nh;
    
    
    // get parameter from config file 
    nh.param<std::string>("mapSaveLocation", mapSaveLocation, "/home/lgt/multicamera_ws/src/avp_slam/data/");
    nh.param<double>("odomKeyFramDisThresh", odomKeyFramDisThresh, 1.0);
    nh.param<int>("invalidColorThresh", invalidColorThresh, 60);
    nh.param<bool>("useICP", useICP, true);
    nh.param<double>("icpMaxCorrespondenceDistance", icpMaxCorrespondenceDistance, 20);
    nh.param<int>("icpMaximumIterations", icpMaximumIterations, 100);
    nh.param<double>("icpTransformationEpsilon", icpTransformationEpsilon, 1e-10);
    nh.param<double>("icpEuclideanFitnessEpsilon", icpEuclideanFitnessEpsilon, 0.001);
    nh.param<double>("icpFitnessScoreThresh", icpFitnessScoreThresh, 0.3);
    nh.param<float>("pointCloudLeafSize", pointCloudLeafSize, 0.1);
    nh.param<bool>("useNDT", useNDT, true);
    nh.param<double>("ndtTransformationEpsilon", ndtTransformationEpsilon, 1e-10);
    nh.param<double>("ndtResolution", ndtResolution, 0.1);
    nh.param<double>("ndtFitnessScoreThresh", ndtFitnessScoreThresh, 0.001);
    nh.param<bool>("mapSave", mapSave, true);

    
    ros::Subscriber subcameraCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cameraCloudFrame", 100, cameraCloudHandler);
    ros::Subscriber subSaveMap = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, saveMap_callback);
    
    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 100);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 100);
    pubGlobalFeature=nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 100);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);

    transWorldCurrent=pcl::getTransformation(0, 0, 0,0,0,0);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

         rate.sleep();
    }

    //  save map for relocation
    if(mapSave){
       std::cout << "map save start" << std::endl;
       pcl::PointCloud<PointType>  globalMap=*globalFeatureCloud;
       pcl::io::savePCDFileASCII(mapSaveLocation, *globalFeatureCloud);        
	   std::cout << "map save over" << std::endl;
    }

    std::cout<<"hello slam"<<std::endl;
    return 0;
}



























