# object_on_table_segmenter (BETA)

Author: Felix Jonathan (fjonath1@jhu.edu)

## Prerequisites

To run this code you need:
  - Some point cloud2 publisher
  - ROS
  - PCL 1.7

## Executing

How to run the code:

```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch
```

It is possible to pass some arguments to set the directory data, point cloud input, etc.
Args list:
 - object:	the object folder name without extension. Default: ```cloud_cluster_```
 - pcl_in:	Input point cloud topic name. Default: ```/camera/depth_registered/points```
 - plane_seg_viewer:	See first distance filtered and table segmented pcl. Default: ```false```
 - save_directory:	Location of save directory for the data collection. Default: ```$(find object_on_table_segmenter)/result```
 - save_index: Index of the first segmented cloud.
 - load_table: Load existing `table.pcd` data generated from previous table segmentation result
 - load_table_path: Location of `table.pcd` file to load
 - table_tf: The name of tf frame we are going to use
 - auto_capture: Automatically capture new data after the table has been initialized

`launch/table_seg_params.launch` contains table segmentation parameters that can be set and `launch/object_on_table_segmenter.launch` contains more arguments that can be passed into the code for modifying data capture operation.

## Examples

We use aruco markers that are 90mm across the black part 100mm total width on white outline.

Before starting make sure your sensor has a full and accurate view of the target region free of clutter.


```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch tableTF:=ar_marker_2 use_tf_surface:=true above_table_min:=0.01 object:=blue_bin load_table:=true num_to_capture:=200
```

1. Here on the first run you put the marker on the table/turntable. 

![Use the AR tag and a vertical area above it to define the object region](ar_tag_sets_plane.jpg)

2. press 's' to get the pose of the marker, this is saved to disk
3. remove the marker, place your detection object on the table where the marker was

![place the object where the tag was](scanning_blue_bin_on_turntable.jpg)

4. press 's' again to begin collecting data

```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=link_vert table_tf:=ar_marker_2 load_table:=false num_to_capture:=200
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=link_horizontal table_tf:=ar_marker_2 load_table:=false num_to_capture:=200
```

Results will be placed in the local directory with the following structure

```
~/catkin_ws/src/costar_stack/costar_tools/object_on_table_segmenter
├── data
├── launch
├── link_uniform
├── node_uniform
├── original
├── res
├── result
│   ├── red_bin
│   │   ├── ground_truth
│   │   └── original
│   ├── blue_bin
│   │   ├── ground_truth
│   │   └── original
│   └── red_link
│       ├── ground_truth
│       └── original
├── result_vert_01
├── sander
│   └── original
└── src

```

"original" will contain the full source pcd (pcl point cloud format) files.
"ground_truth" will contain only segmented object data.

Files are written as follows:
```
<date>_<name>_<entry#>_ground_truth.pcd
2016_04_12_11_51_30_blue_bin_400_ground_truth.pcd

<date>_<name>_<entry#>_original.pcd
2016_04_12_11_51_30_blue_bin_400_original.pcd
```

If clustering is enabled each ground truth may have multiple entries. For example,
if you want to collect separate data for multiple objects on the table simultaneously.
Then the gorund truth format will be:

```
<date>_<name>_<entry#>_cluster_<cluster#>_ground_truth.pcd
2016_04_12_11_51_30_blue_bin_400_ground_truth.pcd
```

#### Verify your data

Be sure to verify your data and make sure no incorrect data is present!

`pcl viewer path/to/file.pcd`

You can load everything at once to make your life easier and check for outliers:

`pcl viewer path/to/folder/with/pcds/*`


#### image extraction and segmentation

You can run `pcl_pcd2png` to convert your pcds into images

You can run `pcd2png_segment_gt.cpp` to perform semantic segmentation of your ground truth data and your image data.
