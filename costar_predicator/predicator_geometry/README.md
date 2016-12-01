# Predicator Geometry Plugin

This is for reasoning about spatial relationships. For example:

```
rosservice call /predicator/get_assignment "statement:
  predicate: 'right_of'
  value: 0.0
  confidence: 0.0
  num_params: 0
  params: ['gbeam_link_1/gbeam_link', '*', 'world']
  param_classes: ['']" 
```

Aka, "what, if anything, is to the left of link1 in the world frame?" results in:

```
found: True
values: 
  - 
    predicate: right_of
    value: 0.0
    confidence: 0.0
    num_params: 0
    params: ['gbeam_link_1/gbeam_link', 'gbeam_node_2/gbeam_node', 'world']
    param_classes: []
```

Whereas:

```
rosservice call /predicator/get_assignment "statement:
  predicate: 'in_back_of'
  value: 0.0
  confidence: 0.0
  num_params: 0
  params: ['gbeam_link_1/gbeam_link', '*', 'world']
  param_classes: ['']" 
```

AKA, "what is in back of link1 in the world frame?" results in:

```
found: True
values: 
  - 
    predicate: in_back_of
    value: 0.0
    confidence: 0.0
    num_params: 0
    params: ['gbeam_link_1/gbeam_link', 'gbeam_node_1/gbeam_node', 'world']
    param_classes: []
  - 
    predicate: in_back_of
    value: 0.0
    confidence: 0.0
    num_params: 0
    params: ['gbeam_link_1/gbeam_link', 'gbeam_node_2/gbeam_node', 'world']
    param_classes: []
```
