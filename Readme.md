# Predicator

Predicator is the CoSTAR package for logical statements.

## Using Predicator

### Using Launch Files

There is a package called `predicator_bringup` that will start different predicator modules and the predicator core.

#### Launch the Core Only

```
roslaunch predicator_bringup core.launch
```

#### Launch the Peg Simulation Example

This launch file uses the configuration included to test predicator in the simulation.
It depends on having `lcsr_collab` and optionally `lcsr_spacenav` running, so that there is a Gazebo world containing two Barrett WAM arms, two pegs, a stage, and a ring.

```
roslaunch predicator_bringup pegs_sim_test.launch
```

### Starting Predicator from Rosrun

Start `predicator_core` to listen to predicate statements from modules:

```
rosrun predicator_core core.py
```

Once the core is up and running, you can launch different modules to produce predicates. Keep in mind that for our purposes, predicates are always true statements about the world.

It may be best to build custom launch files for the different predicator modules instead of launching with `rosrun` since each module needs to be carefully configured.

### Instructor Support

Instructor plugins are in the `predicator_plugins` package.
The user interfaces may require `predicator_core` to be running to get a list of possible predicates.

### Provided Services

- **predicator/test_predicate**: determines if a predicate is true
- **predicator/get_assignment**: return the set of possible values for a single missing field
- **predicator/get_possible_assignment**: list the set of all possible values, if you provide an empty id. List of all possible values for a valid single-term predicate (a type) if you provide an id.
- **predicator/get_predicates**: list the set of all predicates currently considered valid
- **predicator/get_value_predicates**: list values published by Predicator modules

## Modules

Predicator modules are the ROS packages that actually perform some kind of analysis and publish that analysis as predicates. They are the source of all information used by Predicator.

### List of Modules

- **predicator_dummy_module**: publishes a bunch of dummy information for testing purposes
- **predicator_collision**: collisions between objects; uses URDFs of objects to determine spatial relationship information.
- **predicator_geometry**: determine object spatial relationships based on positions (TF frames).
- **predicator_occupancy_module**: select a volume, determine if anything enters that volume. 
- **predicator_fake_classification**: publish known object class information. For use with a simulator, when a real object detector isn't in use.
- **predicator_movement**: publish movement information, such as whether an object is approaching another object.
- **predicator_params**: provides a service so that you can set predicates at runtime to save information.

### Module Setup

#### Geometry Module

##### Geometry Module Configuration Example

Nodes like the `predicator_geometry` module can be configured from the ROS parameter server.
It may be best to start them from a launch file, like the example launch file in `predicator_geometry/launch/pegs_geometry_predicates_test.launch`.

```xml
<node name="predicator_geometry_node"
  type="predicator_geometry_node.py"
  pkg="predicator_geometry"
  output="$(arg output)">

  <param name="height_threshold" value="0.1"/>
  <param name="rel_x_threshold" value="0.1"/>
  <param name="rel_y_threshold" value="0.1"/>
  <param name="rel_z_threshold" value="0.1"/>
  <param name="near_2D_threshold" value="0.2"/>
  <param name="near_3D_threshold" value="0.25"/>

  <rosparam param="frames">
    - ring1/ring_link
    - peg1/peg_link
    - peg1/base_link
    - peg2/peg_link
    - peg2/base_link
    - wam/shoulder_yaw_link
    - wam/shoulder_pitch_link
    - wam/upper_arm_link
    - wam/forearm_link
    - wam/wrist_yaw_link
    - wam/wrist_pitch_link
    - wam/wrist_palm_link
    - wam2/shoulder_yaw_link
    - wam2/shoulder_pitch_link
    - wam2/upper_arm_link
    - wam2/forearm_link
    - wam2/wrist_yaw_link
    - wam2/wrist_pitch_link
    - wam2/wrist_palm_link
    - stage_link
  </rosparam>
</node>
```

## Writing a Module

Start with:

```
catkin_create_pkg predicator_custom_module predicator_msgs
```

New modules should publish a list of predicates
(a `predicator_msgs/PredicateList` message) to the appropriate topic.
By default, `predicator_core` will listen to the `predicator/input` topic for information from modules.

Modules need to set the `header.frame_id` field to their node name, indicating where messages are coming from.

#### Example Module

Look at **predicator_dummy_module** for an example of how a module should publish predicate statements.

## Troubleshooting

### Contact

Predicator is maintained by Chris Paxton (cpaxton3@jhu.edu).
