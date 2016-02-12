# Multi Kinect Merge

This is rosnode that can take multiple kinect source, combine them to one point cloud2 and publish it.

## Prerequisites

To run this code you need:
  - Two source of point clouds
  - Two TF frames of same object that can be detected by all kinect view

Before running the merging algorithm, the program needed to have at least one calibration to get the kinect to kinect transformation

## Calibration

To run the calibration, run
```
roslaunch multi_kinect_merge multiKinect.launch mode:=0 <additional args1> <additional args2>  <additional args3>
```

Args list:
- tfNames         : The source of TF of the same object. Tested with AR tag. Separate the tfnames with `,` . Example: ```tfNames:=ar1_0,ar2_0``` for tf name ar1_0 detected by source1 and ar2_0 detected by source2
- pcl_in          : The source of point clouds (not necessarily kinect). Separate between point cloud source topic with `,` . Example: ```pcl_in:=/kinect1/qhd/points,/kinect2/qhd/points```
- tf_only         : Estimate the kinect to kinect transformation only with TF source. Fast and reliable. Set to false to do additional pcl ICP algorithm with TF transform as initial guess (slow and might not be reliable depending on the point cloud source). Default: true. Example ```tf_only:=false```
- mode            : Switch between calibration program and merge program. Set to 0 for calibration and set to 1 for merging point cloud.

The number of tfNames should be equal to number of point cloud sources

## Merge point cloud
To run the point cloud merging algorithm:
```
roslaunch multi_kinect_merge multiKinect.launch <additional args1> <additional args2>
```

Args list:
- pcl_in          : The source of point clouds (not necessarily kinect). Separate between point cloud source topic with `,` . Example: ```pcl_in:=/kinect1/qhd/points,/kinect2/qhd/points```
- debug           : For debugging purposes. Will save the transformed point clouds and the concacenated point cloud into launch folder. Default: false. Example: ```debug:=true```

