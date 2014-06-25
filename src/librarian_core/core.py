#!/usr/bin/env python

import rospy

from librarian_msgs.msg import *
from librarian_msgs.srv import *

from os.path import expanduser

'''
Librarian()
Class containing functions to save and write configuration information to files.
Files are grouped by "types" like Volumes, Robots, Objects, etc.
These are all saved into different folders in a root directory.
'''
class Librarian(object):

    '''
    __init__()
    Loads in the appropriate root parameter for the library of items.
    Advertises services, and loads some information about different items.
    '''
    def __init__(self):
        root = rospy.get_param('/librarian_root','~/.costar/')
        self._root = expanduser(root)
        print self._root
        self._records = {}
        self._save_srv = rospy.Service('librarian/save', Save, self.save)
        self._save_srv = rospy.Service('librarian/load', Load, self.load)
        self._save_srv = rospy.Service('librarian/list', List, self.get_list)

        self.load_records()

    def save(self, req):
        resp = SaveResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        
        return resp

    def load(self, req):
        resp = LoadResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"

        return resp

    def get_list(self, req):
        resp = ListResponse()
        
        return resp

    def write_records(self):
        pass

    def load_records(self):
        pass

if __name__ == '__main__':
    rospy.init_node('librarian_core')

    #spin_rate = rospy.get_param('rate',10)
    #rate = rospy.Rate(spin_rate)

    try:
        lib = Librarian()

        rospy.spin();

    except rospy.ROSInterruptException:
        # shut down the node!
        # finish pending writes/reads
        # save the current state of the world

        pass
