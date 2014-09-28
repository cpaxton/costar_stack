## Usage
Pick a new region (GUI):
```rosrun predicator_8020_module pick_region.py```
Chose 4 points to define the region where thre plates will be located

Detect if plates are in the region
```rosrun predicator_8020_module detect_8020.py```

#Optional parameter: display:=True if you want to display the video and occupancy sensor.
This file sends an "occupied" predicate from "occupancy_sensor" to predicator.