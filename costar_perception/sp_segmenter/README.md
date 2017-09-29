# SP Segmenter

Thank you for your interest at our semantic segmentation software.

This software implements a modified version of the algorithm described in the papers:
  - C. Li, Jonathan Bohren, Eric Carlson, and G. D. Hager, “Hierarchical Semantic Parsing for Object Pose Estimation in Densely Cluttered Scenes,” IEEE International Conference on Robotics Automation (ICRA) , 2016.
  - C. Li, A. Reiter, and G. D. Hager, “Beyond spatial pooling: Fine-grained representation learning in multiple domains,” IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2015.
  - C. Li, J. Boheren, and G. D. Hager, "Bridging the Robot Perception Gap With Mid-Level Vision," International Symposium on Robotics Research (ISRR), 2015.

If you find this software useful, please site the aforementioned papers above in any resulting publication.

This software repository is maintained by:
  - Chi Li (chi_li@jhu.edu)
  - Felix Jonathan (fjonath1@jhu.edu)

## Prerequisites

Code has all been developed and tested with Ubuntu 14.04 / OSX 10.11.x and ROS Indigo. You will need OpenCV 2.4 and OpenCV nonfree.

If you're using standard opencv2 library from ros, you may be able to install the nonfree with:
```
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update 
sudo apt-get install libopencv-nonfree-dev
```

If you receive errors compiling sp_segmenter because of include problem in opencv nonfree headers, uninstall the nonfree package from the apt-get and build opencv2 by yourself with these commands:
```
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 2.4.13.2
mkdir build
cd build
cmake ..
make -j4
sudo make install
``` 

If you want to build the code with pose estimation, you will need to build and install ObjRecRANSAC:
```
git clone https://github.com/jhu-lcsr/ObjRecRANSAC
cd ObjRecRANSAC
mkdir build
cmake ..
make -j4
sudo make install
```

To run this code you need:
  - Sequence of object partial views in the correct lighting conditions
  - Mesh of the object (for ObjRecRANSAC)
  - Feature Dictionary (default is provided in ```data/UW_shot_dict```)
  - SVM training model (sample svm model is provided in ```data/link_node_svm```)

## Training Model

Application for training:

`main_sp_compact`

This has the training script that loads a library that figures out how to separate the different objects you wish to recognize. Put the different object names in different folders as documented in the list of folders.

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


#### Feature scales

CSHOT features work on a scale defined at training time. This must be kept constant from training through to the 
runtime of your algorithm. Scale is very important because if it is too small it will see part of an object and
not get information about larger scales, and if it is too large it will it will run slower and it won't 
get useful information about the boundaries of objects. 

#### Numbering classes

There are two occasions classification occurs. First in an optional "binary classification" 
that separates foreground data you care about from background you aren't worrying about.

After foreground and background are separated, each class of "foreground" data needs to be seaparted
with a multi class SVM as "drill", "wood_block", or "sander" needs to be classified. Each of these 
classes needs to be assigned a class id number.

Assign class ids like the following 4 class example:

1. background
2. drill
3. wood_block
4. sander


## Training using roslaunch
How to train using roslaunch:

```
roslaunch sp_segmenter SPCompact.launch object:=object1,object2,object3 bg_names:=background1,background2,background3
```

By default, this will read all pcd files in the $(find sp_segmenter)/data/training/(object OR background folder) for every object and background that being passed to roslaunch.
Separate every object/background name with `,` 

Args list:

- object		:	Object folder name without extension. Supports multiple object by inserting `,` between object folder name. Default: ```drill```
- bg_names	:	Background folder name without extension. Supports multiple object by inserting `,` between background folder name. Default: ```UR5_2```
- training_folder	:	Training folder directory where the object and background folder can be found. Default: ```$(find sp_segmenter)/data/training/```
- out_fea_path	:	Output fea folder. Default: ```$(arg training_folder)/fea_pool/```
- out_svm_path	:	Output svm folder. Default: ```$(arg training_folder)/svm_pool/```


## Executing

How to run the code (with the default SVM):

```
rosrun sp_segmenter SPSegmenterServer
```

You need to run this from the root of the directory right now.

By default, the segmenter node listens to the ```/camera/depth_registered/points``` topic and publishes its output on the ```points_out``` topic. You can remap these on the command line to deal with different sources.

## Execute using roslaunch

How to run using roslaunch:

```
roslaunch sp_segmenter SPSegmenter.launch
```

By default, this roslaunch is exactly the same as ```rosrun sp_segmenter SPSegmenterNode -p``` except that it can be run without the need to be on the root of sp_segmenter directory. The data folder in default is located in ```"$(find sp_segmenter)/data/"```, and can be modified by changing parameters.launch.

It is possible to pass some arguments to set the object type, input point cloud topic, the segmenter outputs, and some additional parameters that is used by the segmenter. See ```launch/SPServer.launch``` for more ros parameters that can be costumized.

Args list:

- object		:	the object file name without extension. Support multiple object by adding ```,``` between object name. Pay attention to object order if using multiple object by following svm_path object order (Ignore background tag such as UR5).Default: ```drill```
- minConfidence	:	Minimum confidence for object ransac to be considered for pose publishing. Default: ```0.2```
aboveTable  :   Minimum distance from table for object segmentation in meters. Default: ```0.01```
- pcl_in		:	Input point cloud topic name. Default: ```/camera/depth_registered/points```
- pcl_out		:	Output point cloud topic name. Default: ```/SPSegmenterNode/points_out```
- data_path	:	Location of data folder. Default: ```$(find sp_segmenter)/data```
- svm_path	:	SVM folder directory in data folder to be loaded. Default: ```UR5_drill_svm```
- loadTable	:	Setting this arg true will make the program try to load table.pcd located in data folder which will be used for segmenting object above the table. If the program fails to get the table.pcd or this arg set to false, It will redo the table training. Default: `true`
- saveTable	:	Setting this arg true will update table.pcd with new table convex hull. If the code successfully load table.pcd, this arg will have no effect. Default: `true`
- tableTF		:	The name of TF frame that represents the center of table. This arg only used if the program fails to load table.pcd. The program will make box segmentation with box size 1 meters cubic around the TF position. Default: `tableTF`
- gripperTF   :   The name of TF frame of the gripper where the object would approximately be when grabbed. Default: `endpoint_marker`.
- useTF       :   Use TF frames instead of pose array for object pose representation. Default: `true`

Example:

```
roslaunch sp_segmenter SPServer.launch object:=mallet_ball_pein pcl_in:=/kinect_head/qhd/points
```


After one service call, it will constantly publishes TF frames.
The object TF naming convension generated from SPServer is: ```Obj_<the name of object>_<objectIndex>```.

Example (to update the TF frames):

```
rosservice call /SPServer/SPSegmenter
```

To segment grabbed object on the gripper and update a particular object TF (in this case: Obj::link_uniform::1) :
```
rosservice call /SPServer/segmentInGripper Obj::link_uniform::1 link_uniform
```


### debugging

There are several facilities for debugging both the runtime execution performance of sp_segmenter and the detection/pose estimation performance.

To view detection, set the visualization flag to true with 

```
roslaunch sp_segmenter SPServerNode.launch visualization:=true
```


## Python Binding
In addition to using ros for training and doing semantic segmentation, we can also uses python.

We provided a sample code for training in `python_binding/sample_training.py` and a sample code for semantic segmentation in `python_binding/sample_segmentation.py`

### SpCompact parameters
SpCompact is our main library we use for generating svm model.
There are various parameters that can be set for SpCompact:

- setInputPathSIFT	:	Set the path that contains the SIFT dictionary
- setInputPathSHOT	:	Set the path that contains the SHOT dictionary
- setInputPathFPFH	:	Set the path that contains the FPFH dictionary
- setInputTrainingPath	:	Setting the path that contains the object model folder that each contains point clouds
- setObjectNames	:	Set a list of object class names to be trained. This library will load all point cloud files located in `training_path/object_names[1..n]/*.pcd`
- setBackgroundNames	:	Set a list of background class names to be trained. This library will load all point cloud files located in `training_path/background_name[1..n]/*.pcd`
- setOutputDirectoryPathFEA	:	Set the relative location of output directory for extracted features. 
	- This library will automatically generates the output directory if it does not exists or overwrite the contents if it already exists.
- setOutputDirectoryPathSVM	:	Set the relative location of output directory for SVM model. 
	- This library will automatically generates the output directory if it does not exists or overwrite the contents if it already exists.
- setForegroundCC	:	Set the foreground/background CC value
- setMultiCC	:	Set the inter-object class CC value.
- setBackgroundSampleNumber
- setObjectSampleNumber
- setCurOrderMax
- setSkipFeaExtraction	:	Skip feature extraction. Never enable this unless you just want to repeat SVM classification with the same Feature extration parameters. (Optional)
- setSkipBackgroundSVM	:	Enable/Disable building foreground/background SVM classification model. (Optional)
- setSkipMultiSVM	:	Enable/Disable building multiclass SVM classification if it is not needed (Optional)
- startTrainingSVM	:	Starting the feature extraction followed by training. Call this method when all parameters has been set properly.

### SemanticSegmentation parameters
SemanticSegmentation is our main library we use for labeling point cloud and possibly calculating object poses if `ObjRecRANSAC` is used. It contains various parameters that can be costumized to suit user's specific needs.

Main parameters that needs to be set for point cloud classification:

- setDirectorySHOT	:	Set the directory path that contains SHOT dictionary
- setUseMultiClassSVM	:	Enable/Disable using multiclass SVM classification.
- setUseBinarySVM	:	Enable/Disable using foreground/background SVM classification
- setDirectorySVM	:	Set the directory path that contains the binary and/or multiclass SVM model.
- setPointCloudDownsampleValue	:	Set the pointcloud downsampling value when doing classification.
- setHierFeaRatio	:	Set hier fea ratio.
- setUseVisualization	:	Enable/Disable pcl visualization. Enabling this will visualize all point cloud modification step by step from the raw data. Press q / close window in every step to continue the program. (Optional)

Optional parameters for modifying point cloud before doing classification with SVM model:

- setUseCropBox	:	Enable/Disable creating a box at a certain pose. Points located outside of this box will be deleted.
- setCropBoxSize	:	Set the crop box size (in meters)
- setCropBoxPose	:	Set the crop box pose relative to the camera.
- setUseTableSegmentation	:	Enable/Disable table segmentation
	-  If table segmentation is used, the library will use a convex hull generated from table segmentation to construct a prism. Points located outside of this prism will be deleted.
- setCropAboveTableBoundary	:	Set the convex hull extrusion parameter (min and max value above the convex hull).
- loadTableFromFile	:	Load a particular table convex hull point cloud from a file.
- setTableSegmentationParameters	:	Set the plane segmentation parameters (`distance threshold`, `angular threshold`, `min inliers`) for generating a table convex hull. See pcl plane segmentation for more details regarding these parameters.
- getTableSurfaceFromPointCloud	:	Generate the table convex hull from an input point cloud based on table segmentation parameters.

The point cloud modification is processed in this following order: 
Raw point cloud -> Crop boxed point cloud -> Table segmented point cloud -> Foreground/Background SVM -> Multiclass SVM -> Object Pose computation

Main parameters that needs to be set for pose computation:

- setUseComputePose	:	Enable/Disable pose computation
	-  If multi class svm is disabled, ObjRecRANSAC will calculate the poses from foreground point clouds with all models loaded into one ObjRecRANSAC class. Otherwise, ObjRecRANSAC will only calculate poses from an object point cloud with that object model.
- setUseCuda	:	Enable/Disable Cuda for ObjRecRANSAC. Has no effect if there is no cuda library installed in the machine.
- setModeObjRecRANSAC	:	Set the ObjRecRANSAC mode. [0 = `STANDARD_BEST`, 1 = `STANDARD_RECOGNIZE`, 2 = `GREEDY_RECOGNIZE`]
- setMinConfidenceObjRecRANSAC	:	Set the minimum ObjRecRANSAC confidence to be considered as a valid pose.
- addModel	:	Add the model and with a certain ObjRecRANSAC parameters(`pair_width`, `voxel_size`, `scene_visibility`, `object_visibility`). The object model is loaded from `mesh_folder/object_name.vtk` and `mesh_folder/object_name.obj`. 
	- IMPORTANT: Please make sure that the models are added following the ordering of svm name. For example, for `hammer_nail_drill_svm`, `addModel` needs to be called to load hammer model first, then nail model, and finally drill model. Adding the model with wrong order will cause invalid pose computation result.

Optional parameters that can be set for pose computation:

- setUseObjectPersistence	:	Enable/Disable retaining orientation from previous object detection. Useful if object has some symmetric properties.
- setUsePreferredOrientation	:	Enable/Disable reorienting the object orientation as close as possible to a preffered orientation. Useful if object has some symmetric properties.
- setPreferredOrientation	:	Set the quaternion of the preferred orientation.
- addModelSymmetricProperty	:	Set the model symmetric property for object symmetric orientation realignment. 
	- For example, a cube will has symmetry for every 90 degrees in each axes. Therefore, the symmetric property is (90,90,90,"preferred step value(unused parameter)","preferred axis(unused parameter)"). If it is not set, the object is assumed to have no symmetry.
