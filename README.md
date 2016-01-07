# SP Segmenter

The file "main_ros.cpp" is what you run if you want to use this as a ROS node.

## Prerequisites

To run this code you need:
  - Sequence of object partial views in the correct lighting conditions
  - Mesh of the object (for ObjRecRANSAC)
  - Feature Dictionary (default is provided in ```data/UW_shot_dict```)
  
## Training Model

Feature extraction fraining is implemented in ``main_sp_training.cpp```.

This training code will extract LAB, FPFH, and SIFT features. You can choose which of these to use for SVM learning.

These features are all saved to a sparse matrix for the provided data set. When doing SVM training, you choose the features.

SVM learning is done in ```main_sp_svm.cpp.```

## Executing

How to run the code (with the default SVM):

```
rosrun sp_segmenter SPSegmenterNode -p
rosrun sp_segmenter republisher.py
```

You need to run this from the root of the directory right now.

By default, the segmenter node listens to the ```/camera/depth_registered/points``` topic and publishes its output on the ```points_out``` topic. You can remap these on the command line to deal with different sources.

Adding the ```-v``` flag will visualize segmenter results:

```
rosrun sp_segmenter SPSegmenterNode -p -v
```

## Execute using roslaunch

You can also run the code with roslaunch without the need to go to the root of the directory.

'''
roslaunch sp_segmenter SPSegmenter.launch
'''

By default, the segmenter node will search for "drill" object and returns both point cloud of the objects and its poses.
It is possible to pass some arguments to set the object type, input point cloud topic, and the segmenter outputs.
Args list:
object		:	the object file name without extension. Default: "drill"
pcl_in		:	Input point cloud topic name. Default: "/camera/depth_registered/points"
pcl_out		:	Output point cloud topic name. Default: "/SPSegmenterNode/points_out"
poses_out	:	Output poses topic name. Default: "/SPSegmenterNode/POSES_OUT"

Example:
'''
roslaunch sp_segmenter SPSegmenter.launch object:=mallet_ball_pein pcl_in:=/kinect_head/qhd/pointss
'''

