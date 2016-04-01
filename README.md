# SP Segmenter

The file "main_ros.cpp" is what you run if you want to use this as a ROS node.

## Prerequisites

To run this code you need:
  - Sequence of object partial views in the correct lighting conditions
  - Mesh of the object (for ObjRecRANSAC)
  - Feature Dictionary (default is provided in ```data/UW_shot_dict```)
  
## Training Model


Application for training:

`main_sp_compact`

This has the training script that figures out how to separate the different objects you wish to recognize. Put the different object names in different folders as documented in the list of folders.

This training code will perform feature extraction including LAB, FPFH, and SIFT features. You can choose which of these to use for SVM learning. These features are all saved to a sparse matrix for the provided data set. When doing SVM training, you choose the features.

SVM learning is done in ```main_sp_svm.cpp.```

Training data is in seperate folders, one for each class. For example:

```
.
├── costar_link
├── costar_node
├── fea_pool
├── gripper
├── link
├── link_node
├── node
├── sander
├── svm_pool
└── UR5
```

10 directories

Here, "link", "node", and "sander" are classes, and "UR5" is background data.

Example execution for data in ``~/data/primesense``:

```
roslaunch sp_segmenter SPCompact.launch training_folder:=$HOME/data/primesense object:=link,node,sander bg_names:=UR5
roslaunch sp_segmenter SPCompact.launch object:=link,node,sander training_folder:=$HOME/data/primesense bg_sample_num:=100 obj_sasmple_num:=30
```

We use ``bg_sample_num`` to set the number of samples drawn from each negative training data (background data), and ``obj_sample_num`` to determine the number of samples drawn from each foreground (object) partial view.





### handling recognizing specific objects

Objects are divided into "classes" drill, hammer, cube, rod.

Two sections in main for training:

- first binary foreground background classification
- second multiclass object classification

float CC foregroundBackgroundCC binary cc, "C" parameter in SVM algorithm (see papers on Support Vector Machines) this is the weight/cost of misclassifying objects in training data.


parameters:

obj_sample_num, bg_sample_num

Data is resampled in the algorithm so it is important that the weights of the data being classified is appropriate. Therefore it is important to set the number of samples in foreground and the background.

The total number of background data should be equal to the total number of the background data. We randomly sample patches

Example:

A is foreground
B is background

relationship between foreground and background should be the following for obj_samplenum and bg_sample_num:

numObjectTrainingData*ObjSamples = numBackgroundTrainingData*NumBackgroundSamples

Note that there is only one foreground and one background class, so the foreground data consists of all foreground data.





## Training using roslaunch
How to train using roslaunch:

```
roslaunch sp_segmenter SPCompact.launch object:=object1,object2,object3 bg_names:=background1,background2,background3
```

By default, this will read all pcd files in the $(find sp_segmenter)/data/training/(object OR background folder) for every object and background that being passed to roslaunch.
Separate every object/background name with `,` 

Args list:

object		:	Object folder name without extension. Supports multiple object by inserting `,` between object folder name. Default: ```drill```

bg_names	:	Background folder name without extension. Supports multiple object by inserting `,` between background folder name. Default: ```UR5_2```

training_folder	:	Training folder directory where the object and background folder can be found. Default: ```$(find sp_segmenter)/data/training/```

out_fea_path	:	Output fea folder. Default: ```$(arg training_folder)/fea_pool/```

out_svm_path	:	Output svm folder. Default: ```$(arg training_folder)/svm_pool/```


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

By default, this roslaunch is exactly the same as ```rosrun sp_segmenter SPSegmenterNode -p``` except that it can be run without the need to be on the root of sp_segmenter directory. The data folder in default is located in ```"$(find sp_segmenter)/data/"```, and can be modified by changing parameters.launch.

It is possible to pass some arguments to set the object type, input point cloud topic, and the segmenter outputs.
Args list:

object		:	the object file name without extension. Support multiple object by adding ```,``` between object name. Pay attention to object order if using multiple object by following svm_path object order (Ignore background tag such as UR5).Default: ```drill```

minConfidence	:	Minimum confidence for object ransac to be considered for pose publishing. Default: ```0.2```
aboveTable  :   Minimum distance from table for object segmentation in meters. Default: ```0.01```

pcl_in		:	Input point cloud topic name. Default: ```/camera/depth_registered/points```

pcl_out		:	Output point cloud topic name. Default: ```/SPSegmenterNode/points_out```

poses_out	:	Output poses topic name. Default: ```/SPSegmenterNode/POSES_OUT```

data_path	:	Location of data folder. Default: ```$(find sp_segmenter)/data```

svm_path	:	SVM folder directory in data folder to be loaded. Default: ```UR5_drill_svm```

Example:

```
roslaunch sp_segmenter SPSegmenter.launch object:=mallet_ball_pein pcl_in:=/kinect_head/qhd/points
```

## SPServer
Runs exactly the same as SPSegmenter if useTF param is set false, otherwise the sp_segmentation will only run when the service call has made and publish TF frame instead of pose array.

Execute with:

```
roslaunch sp_segmenter SPServer.launch
```

It supports the same args as SPSegmenter launch.
Additional Args list for SPServer:
loadTable	:	Setting this arg true will make the program try to load table.pcd located in data folder which will be used for segmenting object above the table. If the program fails to get the table.pcd or this arg set to false, It will redo the table training. Default: `true`
saveTable	:	Setting this arg true will update table.pcd with new table convex hull. If the code successfully load table.pcd, this arg will have no effect. Default: `true`
tableTF		:	The name of TF frame that represents the center of table. This arg only used if the program fails to load table.pcd. The program will make box segmentation with box size 1 meters cubic around the TF position. Default: `camera_2/ar_marker_0`
gripperTF   :   The name of TF frame of the gripper where the object would approximately be when grabbed. Default: `endpoint_marker`
useTF       :   Use TF frames instead of pose array for object pose representation. Default: `true`


After one service call, it will constantly publishes TF frames.
The object TF naming convension generated from SPServer is: ```Obj::<the name of object>::<objectIndex>```.

Note: If there is no prior table training or there are some changes to the table position, set loadTable to true and place AR tag with code: `0` to the center of the table before running the roslaunch.

Example (to update the TF frames):

```
rosservice call /SPServer/SPSegmenter
```

To segment grabbed object on the gripper and update a particular object TF (in this case: Obj::link_uniform::1) :
```
rosservice call /SPServer/segmentInGripper Obj::link_uniform::1
```
