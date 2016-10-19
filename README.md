# Sequential Scene Parsing

This rosnode will improve object detection pose estimate and help solving occluded object detection based on sequential scene parsing derived from paper:
	1. Hager, G.D. and Wegbreit, B., 2011. Scene parsing using a prior world model. The International Journal of Robotics Research, p.0278364911399340.
	2. Brucker, M., Léonard, S., Bodenmüller, T. and Hager, G.D., 2012, May. Sequential scene parsing using range and intensity information. In Robotics and Automation (ICRA), 2012 IEEE International Conference on (pp. 5417-5424). IEEE.

# Requirements
This package requires these dependencies:
	- bullet 2.83 with extra packages (in OSX installation, do brew install bullet --with-extra)
	- pcl 1.7.2
	- boost 1.59.x

Additionally, this package has a ros compatible bundling. To install this with ros capability:

# Usage