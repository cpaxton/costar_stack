Beetree
=======

A lightweight implementation of Behavior Trees in Python that interfaces with the ROS framework. It is based loosly on the C++ implementation by Alejandro Marzinotto Cos at KTH.  His original code can be found at [here](https://github.com/almc/behavior_trees), and a publication [here](http://www.csc.kth.se/~almc/pdf/unified_bt_framework.pdf). A great overview of behavior trees (from the ROS perspective) can be found [here](http://www.pirobot.org/blog/0030/).

**BEETREE IS STILL UNDER DEVELOPMENT! SOME NODES ARE NOT IMPLEMENTED!**

Not all of the nodes found in Marzinotto's implementation are finished.  This is a work in progress.  If you have any questions, please let me know at futureneer@gmail.com.

![alt text](https://raw.githubusercontent.com/futureneer/beetree/master/beetree.png "Example BeeTree behavior tree")



#### Dependencies
Besides ROS (catkin), Beetree depends on the package `rqt_dot` (https://github.com/jbohren/rqt_dot) for visualization of the behavior trees.

#### Usage
To run an example that generates dot code for a sample tree, run the command:

```bash
rosrun beetree beetree_test.py
```

This will print out a large chunk of graphviz dot code, as well as a string of text that shows the traversal of a _single tick_ through the tree, based on the structure in the image above:

```bash
Executing Root: (root)
Executing Parallel: (para)
Executing Action: (act_detect_object)
  -  Node: act_detect_object returned status: SUCCESS
Executing Sequence: (sec_pick_move_to_bin): current child: cond_found_obj
Executing Condition: (cond_found_obj)
  -  Node: cond_found_obj returned status: SUCCESS
Executing Sequence: (sec_pick_up): current child: act_move_to_obj
Executing Action: (act_move_to_obj)
  -  Node: act_move_to_obj returned status: SUCCESS
Executing Action: (act_grab)
  -  Node: act_grab returned status: SUCCESS
Executing Action: (act_lift)
  -  Node: act_lift returned status: SUCCESS
  -  Node: sec_pick_up returned status: SUCCESS
Executing Action: (act_move)
  -  Node: act_move returned status: SUCCESS
Executing Sequence: (sec_place): current child: act_move_to_bin
Executing Action: (act_move_to_bin)
  -  Node: act_move_to_bin returned status: SUCCESS
Executing Action: (act_release)
  -  Node: act_release returned status: SUCCESS
  -  Node: sec_place returned status: SUCCESS
Executing Action: (act_reset)
  -  Node: act_reset returned status: SUCCESS
  -  Node: sec_pick_move_to_bin returned status: SUCCESS
  -  Node: para returned status: SUCCESS
ROOT: Child returned status: SUCCESS
```

To run an example that publishes the same tree to `rqt_dot` (a lightweight graphviz visualizer), run the following command:

```bash
roslaunch beetree test_beetree.launch
```

This will start the example tree and publish dot code to the `rqt_dot` visualizer.  You will need to put the topic name "/beetree/dot" in the DOTCODE TOPIC field in rqt_dot and click the "Subscribe" button to see it.

**Currently in the example, unimplemented nodes have been hard coded to return SUCCESS.**
