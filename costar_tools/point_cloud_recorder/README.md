Point Cloud Recorder
--------------------

Saves a point cloud whenever it is called. 
Designed to be used by instructor or anything that sends a very simple rosmsg to trigger it.
Exposes a ros service of type `std_srv/Empty` that saves a single point cloud whenever it is called.


To run:

    python point_cloud_recorder.py

When you need to call it just call the ros service `record_camera`.

command line:

    rosservice call record_camera

how to call from python:

(todo)
