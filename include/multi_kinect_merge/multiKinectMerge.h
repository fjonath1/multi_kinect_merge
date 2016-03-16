#ifndef MULTI_KINECT_MERGE_H
#define MULTI_KINECT_MERGE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include "multi_kinect_merge/stringToArrayLoader.h"
#include "multi_kinect_merge/stringVectorArgsReader.h"

class multiKinectMerge
{
    protected:
        ros::NodeHandle nh;
        tf::TransformListener listener;
        ;
    
    private:
        std::vector<ros::Subscriber> pc_sub;
        ros::Publisher pc_pub;
        std::string kinect1_header_frameID;
        bool okStatus, debug;
        unsigned int numberOfPointClouds;
        std::string calibrationLocation;
        std::vector<std::string> listOfPointCloudnames;
        std::vector<std::string> listOfTFkinectNames;
        pcl::PointCloud<pcl::PointXYZRGBA> combinedPCL;
        std::map <std::string, tf::Transform> kinect2kinectTFTransform;
        std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cachedPCL;
        void cachePCL(const sensor_msgs::PointCloud2 &pc);
    ;
    
    public:
        multiKinectMerge(const ros::NodeHandle &nh);
        void publishTFlinks();
        void loadTransformParams();
        bool combinePCL();
    ;
};

#endif