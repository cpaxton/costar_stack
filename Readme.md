# Predicator

Predicator is the CoSTAR package for logical statements.

## Modules

- **predicator_collision**: collisions between objects; uses URDFs of objects to determine spatial relationship information.
- **predicator_geometry**: determine object spatial relationships based on positions (TF frames).
- **predicator_occupancy_module**: select a volume, determine if anything enters that volume. 

## Writing a Module

Start with:

```
catkin_create_pkg predicator_custom_module predicator_msgs
```

New modules should publish a list of predicates
(a `predicator_msgs/PredicateList` message) to the appropriate topic.
By default, predicator_core will listen to the `predicator/input` topic for information from modules.

Modules need to set the `header.frame_id` field to their node name, indicating where messages are coming from.

#### Example Module

Look at **predicator_dummy_module** for an example of how a module should publish predicate statements.

## Using Predicator

## Troubleshooting

### Contact

Predicator is maintained by Chris Paxton (cpaxton3@jhu.edu).
