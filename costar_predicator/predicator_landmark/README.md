# Predicator Landmark Tool

Takes in:
  - geometry_msgs/PoseArray for different landmarks
  - predicator_msgs/PredicateList for predicates that must be true

Finds valid TF frames that match these arguments and publishes the transforms relative to them. All TF namespaced.
