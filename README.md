# Sequential Scene Parsing

This rosnode will improve object detection pose estimate and help solving occluded object detection based on sequential scene parsing derived from paper:
```
@article{hager2011scene,
  title={Scene parsing using a prior world model},
  author={Hager, Gregory D and Wegbreit, Ben},
  journal={The International Journal of Robotics Research},
  volume={30},
  number={12},
  pages={1477--1507},
  year={2011},
  publisher={SAGE Publications Sage UK: London, England}
}
@inproceedings{brucker2012sequential,
  title={Sequential scene parsing using range and intensity information},
  author={Brucker, Manuel and L{\'e}onard, Simon and Bodenm{\"u}ller, Tim and Hager, Gregory D},
  booktitle={Robotics and Automation (ICRA), 2012 IEEE International Conference on},
  pages={5417--5424},
  year={2012},
  organization={IEEE}
}
```
# Requirements
This package requires these dependencies:
 - BulletPhysics 2.83+ with EXTRA packages installed.
   - In OSX installation, do brew install bullet --with-extra
   - In linux installation, configure bullet with BUILD_SHARED_LIBS = ON, BUILD_EXTRAS=ON, INSTALL_EXTRA_LIBS=ON then build it
 - pcl 1.7.2+
 - boost 1.59+

Additionally, this package has a ros compatible bundling. To install this with ros capability:
 - clone `costar_objrec_msgs` and `objrec_hypothesis_msgs` from `costar_stack` package into your `catkin_ws` directory

# Usage
 1. Use obj_convex_decomposition on your input obj mesh file to generate a collision mesh (.bcs). Put the generated file into the mesh directory
 2. Use pcl_mesh_sampling to generate surface sampled point cloud of your input obj mesh file, and put it into the mesh directory.
