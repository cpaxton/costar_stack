# CoSTAR Tools

This section includes the code to manage the MoveIt collision detection and a few other utilities that provide useful services for creating powerful robot programs.

**MoveIt collision detection**: creates a PlanningScene based on detected objects, plus adds a table.

Run with:
```
roslaunch moveit_collision_environment colision_env.launch mesh_source:=$(find moveit_collision_environment)/data/mesh tableTFname:=ar_marker_2 defineParent:=true parentFrameName:=/world
```

**Recording point clouds**: this is used to collect data and scan objects. Run with:

```
rosrun point_cloud_recorder point_cloud_recorder.py _id:=$OBJECT_NAME _camera:=$CAMERA_ID
```

This will expose the ``/record_camera`` rosservice which can be called from a UI node. Files will be created wherever you ran the point cloud recorder.

Roslaunch would look something like:

```xml
<node name="point_cloud_recorder" pkg="point_cloud_recorder" type="point_cloud_recorder.py">
  <param name="id" value="NameOfMyObject"/>
  <param name="camera" value="kinect2"/>
</node>
```
