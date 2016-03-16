#include "multi_kinect_merge/multiKinectMerge.h"

multiKinectMerge::multiKinectMerge(const ros::NodeHandle &nh)
{
    this->nh = nh;
    listOfPointCloudnames = stringVectorArgsReader<std::string>(nh, "pcl_in", "");
    this->nh.param("data_folder",calibrationLocation,std::string(""));
    
    this->nh.param("debug",debug,false);
    
    std::cerr << "Subscribe pcl\n";
    for (std::size_t i = 0; i < listOfPointCloudnames.size(); i++)
    {
        std::cerr << listOfPointCloudnames.at(i) <<std::endl;
        pc_sub.push_back(
                        this->nh.subscribe(listOfPointCloudnames.at(i),1,&multiKinectMerge::cachePCL, this)
                         );
    }
    numberOfPointClouds = listOfPointCloudnames.size();
    loadTransformParams();
    pc_pub = this->nh.advertise<sensor_msgs::PointCloud2> ("combinedPointCloud", 1);
    sleep(1.0);
}

void multiKinectMerge::publishTFlinks()
{
    for (std::size_t i = 1 ; i < listOfTFkinectNames.size(); i++) {
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(kinect2kinectTFTransform[listOfTFkinectNames.at(i)], ros::Time::now(), listOfTFkinectNames[0], listOfTFkinectNames[i]));
    }
}

void multiKinectMerge::loadTransformParams()
{
    okStatus = true;
    kinect2kinectTFTransform[listOfPointCloudnames.at(0)].setIdentity();
    for (std::size_t i = 0; i < listOfPointCloudnames.size() - 1; i++)
    {
        std::stringstream ss;
        ss<< "transform" << i;
        if (nh.hasParam(ss.str()))
        {
            Eigen::Transform<double,3,Eigen::Affine> kinect2kinectTransform = getTransformFromParam<double>(nh,ss.str());
            tf::Transform transform;
            tf::transformEigenToTF(kinect2kinectTransform,transform);
            
            if (i == 0) {
                std::string tmp; nh.getParam(ss.str()+"/source_frame",tmp);
                listOfTFkinectNames.push_back(tmp);
            }
            std::string tmp;
            nh.getParam(ss.str()+"/destination_frame",tmp);
            listOfTFkinectNames.push_back(tmp);
            kinect2kinectTFTransform[tmp]=transform;
            std::cerr << "Kinect " << ss.str() << std::endl << kinect2kinectTransform.matrix() << std::endl;
        }
        else
        {
            okStatus = false;
            std::cerr << "ERROR.\n Fail to get calibration parameter for transforming point cloud." << i << std::endl;
            break;
        }
    }
    std::cerr << "Loading done\n";
}

void multiKinectMerge::cachePCL(const sensor_msgs::PointCloud2 &pc)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cache(new pcl::PointCloud<pcl::PointXYZRGBA>());
    sensor_msgs::PointCloud2 pc2;
    pcl_ros::transformPointCloud(listOfTFkinectNames.at(0), kinect2kinectTFTransform[pc.header.frame_id], pc, pc2);
    pcl::fromROSMsg(pc2,*cache);
    cachedPCL.push_back(cache);
    
    if (debug) {
        std::cerr << "Adding point cloud to cache\n";
        std::stringstream ss;
        ss << calibrationLocation << "pc" << pc.header.frame_id << ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGBA> (ss.str(), *cachedPCL[cachedPCL.size() - 1], true);
    }
}

bool multiKinectMerge::combinePCL()
{
    pcl::PCDWriter writer;
    if (!okStatus) return okStatus;
    
    for (std::size_t i = 0; i < cachedPCL.size(); i++)
    {
        if (debug)
            std::cerr << "Combining point cloud" << i <<"\n";
        
        if (i == 0)
            combinedPCL = *cachedPCL[0];
        else
            combinedPCL += *cachedPCL[i];
    }
    
    if (cachedPCL.size() > 0)
    {
        if (debug)
        {
            std::cerr << "Publishing point cloud...\n";
            writer.write<pcl::PointXYZRGBA> (calibrationLocation+"finalCloud.pcd", combinedPCL, true);
        }
        
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(combinedPCL,output_msg);
        output_msg.header.frame_id = listOfTFkinectNames[0];
        pc_pub.publish(output_msg);
        cachedPCL.clear();
    }
    return okStatus;
}