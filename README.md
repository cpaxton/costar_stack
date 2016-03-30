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
object		:	the object folder name without extension. Default: ```cloud_cluster_```
pcl_in		:	Input point cloud topic name. Default: ```/camera/depth_registered/points```
viewer	    	:	See first distance filtered pcl and save it. Default: ```false```
save_directory	:	Location of save directory. Default: ```$(find object_on_table_segmenter)/result```

save_index = 0

laod_directory

tableTF    : marker we are going to use


## Examples

```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=link_vert_ tableTF:=ar_marker_2 loadTable:=false num_to_capture:=200
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=link_horizontal_ tableTF:=ar_marker_2 loadTable:=false num_to_capture:=200
```
