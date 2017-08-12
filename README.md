# Color Nearest Neighboor Segmenter

This rosnode will perform a color based segmentation for objects with flat color. The color training is performed using k-means clustering of each object color category from their color point clouds. Then the model is used for perfoming nearest neighboor search on input color point cloud.

# Requirements
This package requires these dependencies:
	- PCL 1.7.2+
	- OpenCV 2.4.x
	- ROS Indigo
	- [object_on_table_segmenter](https://github.com/jhu-lcsr/object_on_table_segmenter)

This package has been tested on 14.04 using PCL 1.7.2, OpenCV 2.4.8, and ROS Indigo.

# Usage
Run this node using roslaunch with:
```
roslaunch color_nn_segmenter segmenter.launch
```