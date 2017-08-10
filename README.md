# Color Nearest Neighboor Segmenter

This rosnode will perform a simple color based segmentation using nearest neigboor of each pixel to the pretrained colors. These trained colors are generated using k-means clustering of the input images or color point clouds.

# Requirements
This package requires these dependencies:
	- PCL 1.7.2+
	- OpenCV 2.4.x
	- ROS Indigo

This package has been tested on 14.04 using PCL 1.7.2, OpenCV 2.4.8, and ROS Indigo.

# Usage
Run this node using roslaunch with:
```
roslaunch color_nn_segmenter segmenter.launch
```