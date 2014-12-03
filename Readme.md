# Predicator

Predicator is the CoSTAR package for logical statements.

## Starting Predicator

### Using Launch Files

There is a package called `predicator_bringup` that will start different predicator modules and the predicator core.

#### Launch the Core Only

```
roslaunch predicator_bringup core.launch
```

This will launch the `predicator_params` module as well if `params:=true` is set (it is set by default). This is the service which lets other programs manually configure predicator parameters.

**UPDATE 2014-08-11:** As of today, `predicator_params` has been folded into `predicator_core` and this is no longer necessary.

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

## Querying Predicator

The Predicator core just aggregates predicates from a number of different topics.

Predicator works through a few different services, described below.

### Instructor Support

Instructor plugins are in the `predicator_plugins` package.
The user interfaces may require `predicator_core` to be running to get a list of possible predicates.

### Provided Services

- **predicator/test_predicate**: determines if a predicate is true
- **predicator/get_assignment**: return the set of possible values for a single missing field
- **predicator/get_possible_assignment**: list the set of all possible values, if you provide an empty id. List of all possible values for a valid single-term predicate (a type) if you provide an id.
- **predicator/get_predicates**: list the set of all predicates currently considered valid
- **predicator/get_value_predicates**: list values published by Predicator modules
- **predicator/update_param**: manually set a predicator or remove a predicate; these are intended to be parameters that can be fixed and updated dynamically.
- **predicator/get_sources**: list the possible sources, the ROS nodes that Predicator has heard from
- **predicator/get_predicate_names_by_source**: list the names of predicates from each source ROS node
- **predicator/get_all_predicates_by_source**: return a list of all predicates that might be valid, and a truth assignment, for a given source (BY REQUEST FROM KEL)
- **predicator/get_assignment_names_by_source**: list the assignments to the predicate produced by a given source
- **predicator/get_predicate_names_by_assignment**: list the possible predicates for a given assignment, based on ValidPredicates messages received. Will only report predicates reported from one source.
- **predicator/get_assignment_length** returns the number of parameter assignments for a given predicate, if available. Returns -1 if no length has been reported.

Common usage is to call **test_predicate** with a certain predicate to see if it exists, or **get_assigment** with a certain predicate to see what possible values there are for one of its arguments. To use **get_assignment** in this way, fill out a `predicator_msgs::PredicateStatement` object, but replace one argument with an asterisk (\*). Predicates will be returned for all possible values of this argument.

I provided a helper function in **get_possible_assignments** for one-parameter predicates, which returns all possible values as a string. This is used for classes (ex: getting all possible locations or objects).

#### Getting all matches to a predicate

```
$ rosservice call predicator/get_assignment "statement:
  predicate: 'is_closed'
  value: 0.0
  confidence: 0.0
  num_params: 0
  params: ['*', '', '']
  param_classes: ['']" 
found: True
values: 
  - 
    predicate: is_closed
    value: 0.0
    confidence: 0.0
    num_params: 0
    params: ['wam2', '', '']
    param_classes: []
```

#### Getting a list of sources

```
$ rosservice call predicator/get_sources
data: ['/predicator_fake_class_node', '/predicator_wam2_joint_states_node', '/release_collab_frame2_creator', '/predicator_robot_interaction_node', '/predicator_geometry_node', '/predicator_wam_joint_states_node', '/collab_frame2_creator', '/predicator_movement_node', '/drop_points_publisher']
```

#### Getting predicates by source

```
$ rosservice call predicator/get_predicate_names_by_source "id: '/predicator_wam2_joint_states_node'" 
data: ['is_closed']
```

#### Getting assignments by source

```
$ rosservice call predicator/get_assignment_names_by_source "id: '/predicator_wam2_joint_states_node'" 
data: ['wam2']
```

#### Getting all predicates by source

This is an example from the peg demo.

```
$ rosservice call predicator/get_all_predicates_by_source "id: '/predicator_wam2_joint_states_node'" 
predicates: 
  - 
    predicate: is_closed
    value: 0.0
    confidence: 0.0
    num_params: 1
    params: ['wam2', '', '']
    param_classes: []
is_true: [True]
```
Note that since Predicator doesn't use class information, not all predicates produced by this are guaranteed to be valid!

#### Getting assignment length for a predicate

```
$ rosservice call /predicator/get_assignment_length "predicate: 'is_closed'" 
length: 1
```

## Modules

Predicator modules are the ROS packages that actually perform some kind of analysis and publish that analysis as predicates. They are the source of all information used by Predicator.

### List of Modules

- **predicator_dummy_module**: publishes a bunch of dummy information for testing purposes
- **predicator_collision**: collisions between objects; uses URDFs of objects to determine spatial relationship information.
- **predicator_geometry**: determine object spatial relationships based on positions (TF frames).
- **predicator_occupancy_module**: select a volume, determine if anything enters that volume. 
- **predicator_fake_classification**: publish known object class information. For use with a simulator, when a real object detector isn't in use.
- **predicator_movement**: publish movement information, such as whether an object is approaching another object.
- ~~**predicator_params**: provides a service so that you can set predicates at runtime to save information~~ **[REMOVED 2014-08-11]**.
- **predicator_planning**: computes many of the same predicates as **predicator_geometry** and **predicator_collision**, but also offers a simple randomized motion planning service that attempts to satisfy or negate predicates.

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

Modules need to set the `pheader.frame_id` field to their node name, indicating where messages are coming from.

### Creating a Predicate

Create a `predicator_msgs::PredicateStatement` object and add it to the list of items in the `predicator_msgs::PredicateList` published by each module.

Make sure to fill out the fields:

- `predicate`: the name of the predicate to publish
- `params`: 3-tuple containing the arguments to this predicate
- `num_params`: number of these parameters you are actually using
- `param_classes`: descriptions of the parameters you are using (i.e., "object", "robot" -- class information)
- `confidence`: how accurate this predicate is believed to be (currently not really used for anything)
- `value`: the value associated with a predicate.

Boolean predicates can be given the values `predicator_msgs::PredicateStatement::TRUE`, `predicator_msgs::PredicateStatement::FALSE`, and `predicator_msgs::PredicateStatement::UNKNOWN`.

### Specifying Valid Predicates

You can send a `predicator_msgs::ValidPredicates` object to help specify what types of predicates your modules can publish that are valid.

Fill out the following fields:

- `assignments`: the possible parameter arguments to your predicates (a union of any predicates you publish)
- `predicates`: the normal, boolean predicates you send out
- `valid_predicates`: floating point valued features such as distance, etc. that your module may compute.

#### Example Module

Look at **predicator_dummy_module** for an example of how a module should publish predicate statements.

## Troubleshooting

### Contact

Predicator is maintained by Chris Paxton (cpaxton3@jhu.edu).
