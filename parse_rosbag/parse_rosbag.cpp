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
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char *argv[]){
    std::string valuableTopic = "/currentFeatureInWorld";
    double icpFitnessScoreThresh = 0.3;
    Eigen::Affine3f transWorldCurrent;

    std::vector<pcl::PointCloud<PointType>::Ptr> pointClouds;
    std::vector<ros::Time> pointCloudsTime;

    ros::init (argc, argv, "bag_it");
    rosbag::Bag bag;
    std::string dataFile = "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/";
    bag.open(dataFile+"2022-04-04-18-44-50.bag", rosbag::bagmode::Read);

    // file to save the vertex and edges to
    ofstream myfile;
    myfile.open (dataFile+"output.g2o");

    // possible to look at point clouds?
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");


    float currentX=0;
    float currentY=0;
    float currentZ=0;
    float currentRoll=0;
    float currentPitch=0;
    float currentYaw=0;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();

        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL){
            size_t pos = s->data.find(" ");
            std::string name = s->data.substr(0, pos);
            //std::cout << name << std::endl;
            // TODO parse and extract time and save the times in an array so we can convert to indexes
            myfile << s->data << std::endl;
        }

        sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
        if (input != nullptr){
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*input,*cloud);
            //viewer.showCloud(cloud);
            //std::cout << cloud->size() << std::endl;

            pointClouds.push_back(cloud);
            pointCloudsTime.push_back(time);
        }
    }

    bag.close();
    std::cout<<"done reading "<<pointClouds.size()<<" clouds"<<std::endl;

    int ignoredClouds = 30;
    int cloudIncrement = 100;

    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(20); 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.001);

    for(int j=ignoredClouds;j<pointClouds.size();j+=cloudIncrement){
        // j should be all clouds with a possible transformation from previous clouds i
        pcl::PointCloud<PointType>::Ptr cloud_j = pointClouds[j];
        icp.setInputTarget(cloud_j);
        for(int i=0;i<j-ignoredClouds;i+=cloudIncrement){
            //compare the current cloud(j) to all past clouds(i)
            pcl::PointCloud<PointType>::Ptr cloud_i = pointClouds[i];
            icp.setInputSource(cloud_i);
            
            pcl::PointCloud<PointType>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<PointType>());
            icp.align(*transCurrentCloudInWorld);
            
            double icpScore = icp.getFitnessScore();

            //if the score is good then find the transformation
            if (icp.hasConverged() == false || icpScore > icpFitnessScoreThresh) {
                //std::cout << "ICP locolization failed the score is " << icpScore << std::endl;
            } 
            else {
                //std::cout<<"passed "<<i<<" "<<j<<std::endl;
                transWorldCurrent = icp.getFinalTransformation();
                pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
                myfile << "EDGE_SE2 " << pointCloudsTime[i] << " " << pointCloudsTime[j] << " " << currentX << " " << currentY << " " << currentYaw << " 0.1 0 0 0.1 0 0.1" << std::endl;
                // save in this format: EDGE_SE2 i j x y theta info(x, y, theta)
            }
        }
    }

    myfile.close();

    return 0;
}


