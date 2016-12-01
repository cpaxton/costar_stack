# moveit_collision_environment
This is rosnode package that publishes MoveIt planning scene that contains the collision object and table taht detected thru the TF. <br>
The node will get the object name based on the TF frame id (ie the TF name should have some special format). <br>
The default format for the TF frame id for this rosnode is: Obj::`name of the object`::`index`. <br>
Sample TF name: Obj::link_uniform::1 <br>

The rosnode will publish the planning scene in `/planningscene` topic.

# Installation
Dependencies: No external dependencies
Instructions:
1. Go to catkin workspace src. Example: `cd ~/catkin_ws/src`
2. Clone this repo with `git clone https://github.com/fjonath1/moveit_collision_environment.git`
3. Do catkin_make or `catkin build moveit_collision_environment`

# Executing moveit_collision_environment
To execute this rosnode use:
```
roslaunch moveit_collision_environment collision_env.launch  <optional args1>  <optional args2> <optional args n>
```

The roslaunch support these arguments:
* mesh_source         :   The folder that contain the mesh object files. Default: `$(find moveit_collision_environment)/data/mesh`
* charToFind          :   Signature characters that separates between the name of object inside of the TF name. Default: `::`
* objectNameIdx       :   The position of the object name after charToFind separation. Default: `2`
* tableTFname         :   The TF name for the table. Default: `tableTF`
* renewTable          :   This parameter will be read every update and if it sets true, the Table will be updated. Default: `true`
* defineParent        :   The parameter to define the parent frame the collision object should be relative to. If it is false, the header of collision object will be whatever parent frame the point cloud is. Default: `false`
* parentFrameName     :   The parameter to set the name of collision object parent frame. Only used if defineParent parameter is false. Default: `base_link`
* file_extension      :   The extension that used by all mesh files. Supports assimp importable extension, such as: stl, obj, dae, blend. Default=`stl` 


Example:
* If the TF name format = Obj::`objName`::`index` , charToFind = `::`, the objectNameIdx = `2`
* If the TF name format = `objName`-`index`, charToFind = `-`, the objectNameIdx = `1`.

To update the planning scene, use:
```
rosservice call /planningSceneGenerator/planningSceneGenerator
```

Useful Param: (set to false to disable table update so we can remove the ar Tag)
renewTable          :   This parameter will be read every service call. If it is true, the Table will be updated by reading the tableTF. Default: `true`

To ignore any further change in Table AR tag, use: 
```rosparam set /planningSceneGenerator/renewTable false```
, otherwise use:
```rosparam set /planningSceneGenerator/renewTable true```
to keep updating table collision object for each service call

# How To Use this rosnode
1. Open node with moveit setup. Example: `roslaunch ur5_moveit_config demo.launch`
2. Run this node with `roslaunch moveit_collision_environment collision_env.launch`
3. Have another rosnode that publishes object TF in the specified TF name format and/or put the tableTF object in the middle of the table.
4. Run `rosservice call /planningSceneGenerator/planningSceneGenerator` to update the environment according to TF that correspond to the object naming convention and table TF if available.
5. If removing the tableTF object is necessary or you want to keep the table collision object constant, use `rosparam set /planningSceneGenerator/renewTable false` . This param will be read on each service call and enable/disable the table collision object update.

For example:
```
roslaunch moveit_collision_environment colision_env.launch mesh_source:=$(find moveit_collision_environment)/data/mesh tableTFname:=ar_marker_2 defineParent:=true parentFrameName:=/world
```
