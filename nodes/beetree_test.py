#!/usr/bin/env python
import roslib; roslib.load_manifest('beetree')
import rospy 
from beetree import *

# MAIN #########################################################################
if __name__ == '__main__':
    root = NodeRoot('root','start')
    para = NodeParallel(root,'para','detect_obj_move_to_bin')
    act_detect_object = NodeAction(para,'act_detect_object','detect_obj')
    sec_pick_move_to_bin = NodeSequence(para,'sec_pick_move_to_bin','pick_move_to_bin')
    cond_found_obj = NodeParamCondition(sec_pick_move_to_bin,'cond_found_obj','object_found','test','test')
    sec_pick_up = NodeSequence(sec_pick_move_to_bin,'sec_pick_up','pick_up')
    act_move_to_obj = NodeAction(sec_pick_up,'act_move_to_obj','move_to_object')
    act_grab = NodeAction(sec_pick_up,'act_grab','grab')
    act_lift = NodeAction(sec_pick_up,'act_lift','lift')
    act_move = NodeAction(sec_pick_move_to_bin,'act_move','move_above_bin')
    sec_place = NodeSequence(sec_pick_move_to_bin,'sec_place','place')
    act_move_to_bin = NodeAction(sec_place,'act_move_to_bin','move_to_bin')    
    act_release = NodeAction(sec_place,'act_release','release')
    act_reset = NodeAction(sec_pick_move_to_bin,'act_reset','reset')

    tree_dotcode = root.generate_dot()
    print ''
    print '##### BEETREE GENERATED GRAPHVIZ DOT CODE #####'
    print ''
    print tree_dotcode
    print ''

    root.execute()
    # root.print_subtree()