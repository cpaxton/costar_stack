==Usage==
Pick a new volume (GUI):
```rosrun predicator_occupancy_module pick_volume.py```
The centroid and radius of the volume are added to the parameter server ("occupancy_center", "occupancy_radius")

Detect if the volume is occupied:
```rosrun predicator_occupancy_module detect_volume.py```
Optional parameter: display:=True if you want to display the video and occupancy sensor.
This file sends an "occupied" predicate from "occupancy_sensor" to predicator.