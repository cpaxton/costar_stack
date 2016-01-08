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

## Training using roslaunch (to be tested)
How to train using roslaunch:

```
roslaunch sp_segmenter SPCompact.launch object:=object1,object2,object3 bg_names:=background1,background2,background3
```

By default, this will read all pcd files in the sp_segmenter root directory/data/training/(object OR background) for every object and background that being passed to roslaunch.
Separate every object/background name with `,` 
Args list:
object		:	Object folder name without extension. Supports multiple object by inserting `,` between object folder name. Default: ```drill```
bg_names	:	Background folder name without extension. Default: ```UR5_2```
training_folder	:	Training folder directory where the object and background folder can be found. Default: ```$(find sp_segmenter)/data/training/```
out_fea_path	:	Output fea folder. Default: ```fea_pool```
out_svm_path	:	Output svm folder. Default: ```svm_pool```


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

How to run using roslaunch:

```
roslaunch sp_segmenter SPSegmenter.launch
```

By default, this roslaunch is exactly the same as ```rosrun sp_segmenter SPSegmenterNode -p``` except that it can be run without the need to be on the root of sp_segmenter directory.
It is possible to pass some arguments to set the object type, input point cloud topic, and the segmenter outputs.
Args list:
object		:	the object file name without extension. Default: ```drill```
pcl_in		:	Input point cloud topic name. Default: ```/camera/depth_registered/points```
pcl_out		:	Output point cloud topic name. Default: ```/SPSegmenterNode/points_out```
poses_out	:	Output poses topic name. Default: ```/SPSegmenterNode/POSES_OUT```

Example:

```
roslaunch sp_segmenter SPSegmenter.launch object:=mallet_ball_pein pcl_in:=/kinect_head/qhd/points
```

