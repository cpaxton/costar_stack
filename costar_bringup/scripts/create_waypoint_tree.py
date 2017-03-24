#!/usr/bin/env python

import datetime
import time

num_waypoints = 203
start_waypoint = 34
record_waypoint = 1
scan_start = 0

waypoint_tree = """name: scan-3
tree:
  children:
  - children:
    - children:
      - children: []
        name: basic_grasp_0
        save_info:
          generator_info:
            label: {field_name: Label, type: NamedField, value: ''}
            name: {field_name: Name, type: NamedField, value: basic_grasp_0}
            wait_finish: {field_name: Wait, type: NamedField, value: '1'}
          name: basic_grasp_0
          node_type: ACTION
          plugin_name: Basic Grasp
      - children: []
        name: close_s_gripper_0
        save_info:
          generator_info:
            label: {field_name: Label, type: NamedField, value: ''}
            name: {field_name: Name, type: NamedField, value: close_s_gripper_0}
            wait_finish: {field_name: Wait, type: NamedField, value: '1'}
          name: close_s_gripper_0
          node_type: ACTION
          plugin_name: Close S Gripper
      - children:
        - children: []
          name: move_to_waypoint_0
          save_info:
            generator_info:
              acc: {value: 0.75}
              label: {field_name: Label, type: NamedField, value: ''}
              name: {field_name: Name, type: NamedField, value: move_to_waypoint_0}
              vel: {value: 0.75}
              waypoint_name: {value: calib--4}
            name: move_to_waypoint_0
            node_type: ACTION
            plugin_name: Move to Waypoint
        - children: []
          name: move_to_waypoint_1
          save_info:
            generator_info:
              acc: {value: 0.75}
              label: {field_name: Label, type: NamedField, value: ''}
              name: {field_name: Name, type: NamedField, value: move_to_waypoint_1}
              vel: {value: 0.75}
              waypoint_name: {value: calib--5}
            name: move_to_waypoint_1
            node_type: ACTION
            plugin_name: Move to Waypoint
        name: selector_0
        save_info:
          generator_info:
            label: {field_name: Label, type: NamedField, value: ''}
            name: {field_name: Name, type: NamedField, value: selector_0}
          name: selector_0
          node_type: LOGIC
          plugin_name: Selector
      - children:"""

try:
    i = 0
    while i < num_waypoints:
        # move_to_waypoint_39
        # record_data_0
        # scan04--0
        waypoint_tree += """
        - children: []
          name: move_to_waypoint_%s
          save_info:
            generator_info:
              acc: {value: 0.75}
              label: {field_name: Label, type: NamedField, value: ''}
              name: {field_name: Name, type: NamedField, value: move_to_waypoint_%s}
              vel: {value: 0.75}
              waypoint_name: {value: scan04--%s}
            name: move_to_waypoint_%s
            node_type: ACTION
            plugin_name: Move to Waypoint
        - children: []
          name: record_data_%s
          save_info:
            generator_info:
              label: {field_name: Label, type: NamedField, value: ''}
              name: {field_name: Name, type: NamedField, value: record_data_%s}
              wait_finish: {field_name: Wait, type: NamedField, value: '1'}
            name: record_data_%s
            node_type: ACTION
            plugin_name: Record Data""" % (i + start_waypoint, i + start_waypoint, i + scan_start, i + start_waypoint, i + record_waypoint, i + record_waypoint, i + record_waypoint)
        i += 1

    waypoint_tree += """
        name: sequence_1
        save_info:
          generator_info:
            label: {field_name: Label, type: NamedField, value: ''}
            name: {field_name: Name, type: NamedField, value: sequence_1}
          name: sequence_1
          node_type: LOGIC
          plugin_name: Sequence
      name: sequence_0
      save_info:
        generator_info:
          label: {field_name: Label, type: NamedField, value: ''}
          name: {field_name: Name, type: NamedField, value: sequence_0}
        name: sequence_0
        node_type: LOGIC
        plugin_name: Sequence
    name: repeat_0
    save_info:
      generator_info:
        label: {field_name: Label, type: NamedField, value: ''}
        name: {field_name: Name, type: NamedField, value: repeat_0}
        repeat: {field_name: Repeat (-1) for infinity, type: NamedField, value: '-1'}
      name: repeat_0
      node_type: LOGIC
      plugin_name: Repeat
  name: root
  save_info:
    generator_info:
      label: {field_name: Label, type: NamedField, value: ''}
      name: {field_name: Name, type: NamedField, value: root}
    name: root
    node_type: LOGIC
    plugin_name: Root
"""

    text_file = open("Output.txt", "w")

    text_file.write(waypoint_tree)

    text_file.close()
except Exception, e:
    print e
finally:
    print "at the end of program"
