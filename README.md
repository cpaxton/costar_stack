beetree
=======

A lightweight implementation of Behavior Trees in Python that interfaces with the ROS framework.

BEETREE IS STILL UNDER DEVELOPMENT. SOME NODES ARE NOT IMPLEMENTED.

To run an example that generates dot code for a sample tree, run the command:

```bash
rosrun beetree beetree_test.py
```

To run an example that publishes the same tree to rqt_dot (a lightweight graphviz visualizer), run the following command:

```bash
roslaunch beetree test_beetree.launch
```

This will start the example tree and publish dot code to the rqt_visualizer.  You will need to put the topic name '/beetree/dot' in the DOTCODE TOPIC field in rqt_dot and click the 'Subscribe' button to see it.
