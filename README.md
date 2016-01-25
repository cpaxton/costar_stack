# moveit_collision_environment
This is rosnode package that publises collision object based to be added to MoveIt planning scene.
The node will get the object name based on the TF frame id (so the TF name should have some custom format).
The default format for the TF frame id for this rosnode is: Obj::<name of the object>::<index>.
Sample TF name: Obj::link_uniform::1

# Executing moveit_collision_environment
To execute this rosnode use:
```
roslaunch moveit_collision_environment collision_env.launch
```

The roslaunch support these arguments:
mesh_source         :   The folder that contain the mesh object files. Default: ```$(find moveit_collision_environment)/data/mesh```
charToFind          :   The Character that separates between the name of object inside of the TF name. Default: ```::```
objectNameIdx       :   The position of the object name after charToFind separation. Default: ```2```

Example:
If the TF name format = Obj::objName::index , charToFind = "::", the objectNameIdx = 2
If the TF name format = objName-index, charToFind = "-", the objectNameIndex = 1.
