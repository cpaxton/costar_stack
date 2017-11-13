# Sequential Scene Parsing

This rosnode will improve object detection pose estimate and help solving occluded object detection based on sequential scene parsing derived from paper:
```
@article{jonathan2017temporal,
  title={Temporal and Physical Reasoning for Perception-Based Robotic Manipulation},
  author={Jonathan, Felix and Paxton, Chris and Hager, Gregory D},
  journal={arXiv preprint arXiv:1710.03948},
  year={2017}
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

# Directory Layout
This is the working directory layout for sequential_scene_parsing. The mesh folder contains the physics property list of objects and both the surface pointcloud and the simplified mesh generated from convex decomposition of each object.
```
~/catkin_ws/src/sequential_scene_parsing
├── include
├── launch
├── mesh
│   ├── object_property_database.yaml
│   ├── object_property_database_1.yaml
│   ├── object_property_database_2.yaml
│   ├── ...
│   ├── object_property_database_n.yaml
│   ├── object_1.bcs
│   ├── object_1.pcd
│   ├── object_2.bcs
│   ├── object_2.pcd
│   ├── ...
│   ├── object_n.bcs
│   ├── object_n.pcd
│   └── table.pcd
├── msg
├── script
├── src
├── srv
├── tool
├── srv
└── unit_test
```

# ROS Usage
 1. Generate the simplified mesh for each object using `obj_convex_decomposition`, by running `rosrun sequential_scene_parsing obj_main_decomposition input_file.obj output_file n_clusters concavity invert add_extra_distance_points add_neighbours_distance_points add_faces_points (optional)max_hull_vertices`. For example, if the input mesh filename is `object_1.obj`, the simplified mesh can be generated with `rosrun sequential_scene_parsing obj_main_decomposition object_1.obj object_1 2 100 0 0 0 0`. Please refer to [HACD parameters guide](http://kmamou.blogspot.com/2011/11/hacd-parameters.html) in order to determine the appropriate setting for `n_clusters`, `concavity`, `invert`, `add_extra_distance_points`, `add_neighbours_distance_points`, `add_faces_points`, `max_hull_vertices`.h directory. In general, when the simplified mesh generation is successful, the resulting simplfied mesh's file size is always has smaller than the original mesh and usually smaller than 100kB.
 2. Generate the surface point cloud of the object mesh using `pcl_mesh_sampling -leaf_size 0.003 object_1.obj object_1.pcd`. This surface point cloud will be used to calculate how the estimated pose matches with the input segmented pont cloud, bigger leaf size may improve the accuracy of the pose, but will cause slower pose computation process. Setting the leaf size to `0.003` works reasonably well for most cases.
 3. Move the generated simplified mesh `object_1.bcs` and surface point cloud `object_1.pcd` to the mesh folder.
 4. Set the physics parameters of objects that will be present in the scene on `object_property_database.yaml`.
 5. If the background point cloud topic which publish the background cloud of the workspace is not available, the table surface point cloud (`table.pcd`) where objects will be placed need to be captured and then put on mesh folder. 
 6. Launch the sequential scene parsing by running `roslaunch sequential_scene_parsing ros_scene.launch`. Please refer to [Scene parsing roslaunch parameter guide](launch/ros_scene.launch) for available roslaunch parameters.


