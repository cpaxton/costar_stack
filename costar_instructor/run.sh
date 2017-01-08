# Robot
kel@taanab:~$ roslaunch simple_ur_driver simple_ur_test.launch
kel@taanab:~$ roslaunch instructor_core test_landmark.launch 
kel@taanab:~$ rosrun rqt_gui rqt_gui 
kel@taanab:~$ roslaunch instructor_core working_instructor.launch
kel@taanab:~$ roslaunch instructor_core standalone.launch 
kel@taanab:~$ rosrun rviz rviz

# Main
kel@taanab:~$ roscore
kel@taanab:~$ roslaunch audri audri_sound_server.launch 
kel@taanab:~$ roslaunch predicator_bringup core.launch
# Modules
kel@taanab:~$ roslaunch instructor_modules foot_pedal_actuator.launch 
kel@taanab:~$ roslaunch predicator_occupancy_module pick_volume.launch 
# Gripper
kel@taanab:~$ roslaunch robotiq_c_model_control CModelTeleop.launch
kel@taanab:~$ roslaunch instructor_predicate analyzers.launch 
# Perception
kel@taanab:~$ roslaunch instructor_core shoulder_camera.launch 
kel@taanab:~$ roslaunch instructor_core shoulder_semi_static.launch 
kel@taanab:~$ roslaunch instructor_core shoulder_alvar.launch 
kel@taanab:~$ roslaunch instructor_core shoulder_smooth.launch 
kel@taanab:~$ rosrun instructor_core shoulder_calibration.py 
