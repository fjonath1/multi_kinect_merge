//
//  main_multiKinect.cpp
//  
//
//  Created by Felix Jo on 2/2/16.
//
//
#include <stdio.h>
#include "multi_kinect_merge/multiKinectCalibration.h"
#include "multi_kinect_merge/multiKinectMerge.h"

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cachedPCL;
std::size_t iterationStep = 0;


void cachePCL(const sensor_msgs::PointCloud2 &pc)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cache(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(pc,*cache);
    std::cerr << "Adding point cloud to cache\n";
    cachedPCL.push_back(cache);
    iterationStep++;
}

int main(int argc, char** argv)
{
    // initialize ros node
    ros::init(argc,argv, "multiKinect");
    ros::NodeHandle nh ("~");
    int mode;
    nh.param("mode",mode,0);
    
    switch (mode) {
        case 0: //calibrate
        {
            std::cerr << "mode: Calibrate\n";
            multiKinectCalibration multikinectCalib(nh);
            
            std::cerr << "Getting all data\n";
            multikinectCalib.getData();
            
            std::cerr << "Subscribe pcl done\n";
            multikinectCalib.calibrate();
            std::cerr << "Calibrated\n";
            ros::shutdown();
            break;
        }
        default: //merge
        {
            std::cerr << "mode: Combine Clouds\n";
            bool firstRun = true;
            multiKinectMerge multikinect(nh);
            while (ros::ok()) {
                if (firstRun)
                    std::cerr << "TF published\n";
                multikinect.publishTFlinks();
                
                if (!multikinect.combinePCL()) break;
                
                if (firstRun)
                {
                    std::cerr << "Point cloud published\n";
                    firstRun = false;
                }
                
                ros::spinOnce();
            }
            break;
        }
    }
    return 0;
}