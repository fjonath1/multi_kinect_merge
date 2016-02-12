#include "multi_kinect_merge/multiKinectCalibration.h"

multiKinectCalibration::multiKinectCalibration(const ros::NodeHandle &nh)
{
    this->nh = nh;
    listOfTFnames = stringVectorArgsReader<std::string>(nh, "tfNames", "");
    listOfPointCloudnames = stringVectorArgsReader<std::string>(nh, "pcl_in", "");
    this->nh.param("tf_only",useTFonly,false);
    this->nh.param("data_folder",dumpLocation,std::string(""));
    std::cerr << "use tf only: " << useTFonly << std::endl;
    if (!useTFonly)
    {
        std::cerr << "Subscribe pcl\n";
        for (std::size_t i = 0; i < listOfPointCloudnames.size(); i++)
        {
            std::cerr << listOfPointCloudnames.at(i) <<std::endl;
            pc_sub.push_back(
                            this->nh.subscribe(listOfPointCloudnames.at(i),1,&multiKinectCalibration::cachePCL, this)
                             );
        }
    }
    sleep(1.0);
//    getData();
}

void multiKinectCalibration::getData()
{
    iterationStep = 0;
//    std::string relativeTo = listOfTFnames.at(0);
    kinect2kinectTransform.resize(listOfTFnames.size() - 1);
    ros::Rate r(2);
    
    Eigen::Transform<double,3,Eigen::Affine> kinectReference;
    ros::spinOnce();
    if (!useTFonly)
    {
        while ( iterationStep < listOfTFnames.size() && ros::ok())
        {
            std::cerr << "Waiting for point clouds... \n";
            r.sleep();
            ros::spinOnce();
        }
        showPCL();
        std::cerr << "Subscribe pcl done" << std::endl;
    }
    sleep(1.0);
    std::cerr << listener.allFramesAsString() << std::endl;
    // listen to all tf Frames
    
    // get the transformation between kinects
    for (std::size_t i = 0; i < listOfTFnames.size(); i++)
    {
        tf::StampedTransform transform;
        std::string parentFrame;
        listener.getParent(listOfTFnames.at(i),ros::Time(0),parentFrame);
        listOfPointCloudnameHeaders.push_back(parentFrame);
        
        std::cerr << "Lookup transform: "<< listOfTFnames.at(i) << " with parent: "<< parentFrame <<std::endl;
        listener.waitForTransform(parentFrame,listOfTFnames.at(i),ros::Time(0),ros::Duration(5.0));
        listener.lookupTransform(parentFrame,listOfTFnames.at(i),ros::Time(0),transform);
        Eigen::Transform<double,3,Eigen::Affine> tmpTransform;
        tf::transformTFToEigen(transform,tmpTransform);
        
//        geometry_msgs::TransformStamped msg;
//        transformStampedTFToMsg(transform, msg);
//        Eigen::Translation<float,3> translation(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z);
//        Eigen::Quaternion<float> rotation(msg.transform.rotation.w,
//                                          msg.transform.rotation.x,
//                                          msg.transform.rotation.y,
//                                          msg.transform.rotation.z);
//        std::cerr << "tmp:\n" << tmpTransform.matrix() << std::endl;

        if (i == 0) kinectReference = tmpTransform;
        else kinect2kinectTransform[i-1] = kinectReference * tmpTransform.inverse();
    }
//    std::cerr << "Kinect Ref:\n" << kinectReference.matrix() << std::endl;
}

void multiKinectCalibration::calibrate()
{
    if (!useTFonly) {
        std::cerr << "Available PCL: "<<iterationStep << " " << listOfPointCloudnames.size() << std::endl;
        for (std::size_t i = 1; i < listOfPointCloudnames.size(); i++)
        {
                        // refine the transformation with ICP
            std::cerr << "Aligning kinect" << 1 << " with kinect" << i+1 << std::endl;
            std::cerr << "Initial Pose est:\n" << kinect2kinectTransform[i-1].matrix() << std::endl;
            
            pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
            icp.setInputCloud(cachedPCL[i]);
            icp.setInputTarget(cachedPCL[0]);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBA>());
            
            std::cerr << "Starting ICP between kinect" << 1 << " with kinect" << i+1 << std::endl;
            icp.align(*Final, kinect2kinectTransform[i-1].inverse().matrix().cast<float>());
            
            std::cerr << "ICP has converged. Final transformation:\n";
            std::cerr << icp.getFinalTransformation().inverse() << std::endl;
            // dump the ICP transformation
            kinect2kinectTransform[i - 1] = icp.getFinalTransformation().inverse().cast<double>();
            
            std::stringstream ss;
            ss << dumpLocation << "Aligned" << i << ".pcd";
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZRGBA> (ss.str(), *Final, true);
            showPCL(Final);
        
        }
    }
    else std::cerr << "using tf only";
    dumpParams();
}

void multiKinectCalibration::showPCL()
{
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    for (int i = 0; i < cachedPCL.size(); i++) {
        std::cerr << "Visualizing pcl " << i <<std::endl;
        viewer->removeAllPointClouds();
//        viewer->addPointCloud(input, "whole_scene");
        viewer->addPointCloud(cachedPCL.at(i), "whole_scene");
        viewer->spinOnce (300);
        viewer->removeAllPointClouds();
    }
}

void multiKinectCalibration::showPCL(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input)
{
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    std::cerr << "Visualizing aligned pcl\n";
    viewer->removeAllPointClouds();
    viewer->addPointCloud(input, "whole_scene");
    viewer->spin();
    viewer->removeAllPointClouds();
}

void multiKinectCalibration::cachePCL(const sensor_msgs::PointCloud2 &pc)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cache(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(pc,*cache);
    std::cerr << "Adding point cloud to cache\n";
    cachedPCL.push_back(cache);
    
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << dumpLocation << iterationStep << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss.str(), *cachedPCL[iterationStep], true);
    iterationStep++;
}

void multiKinectCalibration::dumpParams()
{
    std::cerr << "Dumping transforms \n";
    //publish all parameters
    for (std::size_t i = 0; i < kinect2kinectTransform.size(); i++)
    {
        std::stringstream ss;
        ss << "TransformList/transform"<< i;
        nh.setParam(ss.str()+"/source_frame",listOfPointCloudnameHeaders[0]);
        nh.setParam(ss.str()+"/destination_frame",listOfPointCloudnameHeaders[i + 1]);
        publishICPparams(nh, kinect2kinectTransform[i],ss.str());
    }
    
    //dump all parameters
    std::stringstream ss;
    ss << "rosparam dump " << dumpLocation << "kinect2kinectTransform.yaml" <<
     " " << ros::this_node::getName() << "/TransformList";

    system(ss.str().c_str());
}