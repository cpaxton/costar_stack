
# Collecting Additional Data

Goal: to collect data for the white bin in which we will be building our tower.

We run the object on table segmenter to collect this data:
```
roslaunch object_on_table_segmenter object_on_table_segmenter.launch object:=white_object table_tf:=ar_marker_2 load_table:=true auto_capture:=false
```

## Outputs

This produces a set of data in the local folder. These point clouds should include the pixel values that get grouped together as "white."

## Color Segmenter

We want to use the color segmenter for our block-stacking task.
