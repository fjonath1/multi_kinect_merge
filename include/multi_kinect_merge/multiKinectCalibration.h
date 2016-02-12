#ifndef MULTI_KINECT_CALIBRATION_H
#define MULTI_KINECT_CALIBRATION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "multi_kinect_merge/stringToArrayLoader.h"
#include "multi_kinect_merge/stringVectorArgsReader.h"

class multiKinectCalibration
{
    protected:
        ros::NodeHandle nh;
        tf::TransformListener listener;
        ;
    
    private:
        std::vector<std::string> listOfTFnames;

        std::vector<ros::Subscriber> pc_sub;
//        ros::Subscriber pc_sub1,pc_sub2;
        std::vector <Eigen::Transform<double,3,Eigen::Affine> > kinect2kinectTransform;
        std::size_t iterationStep;
        std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cachedPCL;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        void showPCL();
        void showPCL(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input);
        std::string dumpLocation;
        bool useTFonly;
    ;
    
    public:
        multiKinectCalibration(const ros::NodeHandle &nh);
        std::vector<std::string> listOfPointCloudnames;
        std::vector<std::string> listOfPointCloudnameHeaders;
        void getData();
        void calibrate();
        void cachePCL(const sensor_msgs::PointCloud2 &pc);
        void dumpParams();
    ;
};

#endif