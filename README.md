Beetree
=======

A lightweight implementation of Behavior Trees in Python that interfaces with the ROS framework. It is based loosly on the C++ implementation by Alejandro Marzinotto Cos at KTH.  His original code can be found at [github.com/almc/behavior_trees](www.github.com/almc/behavior_trees), and a publication [here](http://www.csc.kth.se/~almc/pdf/unified_bt_framework.pdf). A great overview of behavior trees can be found [here](http://www.pirobot.org/blog/0030/). 

![alt text](https://raw.githubusercontent.com/futureneer/beetree/master/beetree.png "Example BeeTree behavior tree")


**BEETREE IS STILL UNDER DEVELOPMENT! SOME NODES ARE NOT IMPLEMENTED!**

#### Dependencies
Besides ROS (catkin), Beetree depends on the package `rqt_dot` (https://github.com/jbohren/rqt_dot) for visualization of the behavior trees.

#### Usage
To run an example that generates dot code for a sample tree, run the command:

```bash
rosrun beetree beetree_test.py
```

This will print out a large chunk of graphviz dot code, as well as a string of text that shows the traversal of a _single tick_ through the tree.

To run an example that publishes the same tree to `rqt_dot` (a lightweight graphviz visualizer), run the following command:

```bash
roslaunch beetree test_beetree.launch
```

This will start the example tree and publish dot code to the `rqt_dot` visualizer.  You will need to put the topic name "/beetree/dot" in the DOTCODE TOPIC field in rqt_dot and click the "Subscribe" button to see it.

**Currently in the example, unimplemented nodes have been hard coded to return SUCCESS.**
