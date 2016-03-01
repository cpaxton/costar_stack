# moveit_collision_environment
This is rosnode package that publishes MoveIt planning scene that contains the collision object and table taht detected thru the TF. <br>
The node will get the object name based on the TF frame id (ie the TF name should have some special format). <br>
The default format for the TF frame id for this rosnode is: Obj::`name of the object`::`index`. <br>
Sample TF name: Obj::link_uniform::1 <br>

The rosnode will publish the planning scene in `/planningscene` topic.

# Executing moveit_collision_environment
To execute this rosnode use:
```
roslaunch moveit_collision_environment collision_env.launch
```

The roslaunch support these arguments:
* mesh_source         :   The folder that contain the mesh object files. Default: ```$(find moveit_collision_environment)/data/mesh```
* charToFind          :   The Character that separates between the name of object inside of the TF name. Default: ```::```
* objectNameIdx       :   The position of the object name after charToFind separation. Default: ```2```
* tableTFname         :   The TF name for the table. Default: ```tableTF```
* renewTable          :   This parameter will be read every update and if it sets true, the Table will be updated. Default: `true`
* defineParent        :   The parameter to define the parent frame the collision object should be relative to. If it is false, the header of collision object will be whatever parent frame the point cloud is. Default: `false`
* parentFrameName     :   The parameter to set the name of collision object parent frame. Only used if defineParent parameter is false. Default: `base_link`

Example:
* If the TF name format = Obj::`objName`::`index` , charToFind = `::`, the objectNameIdx = `2`
* If the TF name format = `objName`-`index`, charToFind = `-`, the objectNameIdx = `1`.

To update the planning scene, use:
```
rosservice call /planningSceneGenerator/planningSceneGenerator
```