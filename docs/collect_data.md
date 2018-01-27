
# Collecting Additional Data

Goal: to collect data for the white bin in which we will be building our tower.

We run the object on table segmenter to collect this data:
```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=white_object table_tf:=ar_marker_2 load_table:=true auto_capture:=false
```

## List of Commands

Bring these up in separate windows (I'm working on a script for this):
```
roscore
roslaunch costar_bringup ur5_c_model.launch use_scene_parsing_pose:=true \
  use_external_segmenter:=true
roslaunch color_nn_segmenter costar_segmenter.launch load_table:=true \
  load_existing_model:=true model_name:=rgby_wood_white_bin \
  background_labels:=wood_block,white_bin \
  foreground_labels:=blue_block,green_block,red_block,yellow_block \
  kmeans_point_per_model:=2 table_tf:=ar_marker_2
roslaunch sp_segmenter colored_block.launch
roslaunch sequential_scene_parsing block_scene.launch best_hypothesis_only:=true
```

## Outputs

This produces a set of data in the local folder. These point clouds should include the pixel values that get grouped together as "white."

## Color Segmenter

We want to use the color segmenter for our block-stacking task.
