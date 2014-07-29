#!/usr/bin/env python

import rospy
import os

import roslib
roslib.load_manifest("rosparam")
import rosparam

from librarian_msgs.msg import *
from librarian_msgs.srv import *

from os.path import expanduser
from os.path import join

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
    def __init__(self, start_srvs=True):
        root = rospy.get_param('~librarian_root','~/.costar/')
        self._root = expanduser(root)
        print "Librarian working directory: %s"%(self._root)

        self._records = {}

        if start_srvs == True:
            self._save_srv = rospy.Service('librarian/save', Save, self.save)
            self._load_srv = rospy.Service('librarian/load', Load, self.load)
            self._list_srv = rospy.Service('librarian/list', List, self.get_list)
            self._load_param_srv = rospy.Service('librarian/load_params', LoadParams, self.load_params)
            self._add_type_srv = rospy.Service('librarian/add_type', AddType, self.add_type)
            self._get_path_srv = rospy.Service('librarian/get_path', GetPath, self.create_path)

        self.init()
        self.load_records()

    '''
    create_path()
    Create a path
    '''
    def create_path(self, req):
        resp = GetPathResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)
            filename = join(path, req.id)

            if not os.path.exists(path):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.NO_SUCH_TYPE
                resp.status.info = "Type %s does not exist!"%(req.type)
            else:
                resp.path = filename

        return resp


    '''
    init()
    If the library directory specified does not exist, set one up.
    '''
    def init(self):
        if not os.path.exists(self._root):
            os.mkdir(self._root)

    def delete(self, req):
        resp = DeleteResponse()

         if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)
            filename = join(path, req.id)

            if not os.path.exists(path):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.NO_SUCH_TYPE
                resp.status.info = "Type %s does not exist!"%(req.type)
            else:
                resp.status.result = Status.SUCCESS
        
        return resp


       

    '''
    save()
    Save text to a file.
    '''
    def save(self, req):
        resp = SaveResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)
            filename = join(path, req.id)

            if not os.path.exists(path):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.NO_SUCH_TYPE
                resp.status.info = "Type %s does not exist!"%(req.type)
            else:

                # get the operation
                if req.operation == SaveRequest.APPEND:
                    op = 'a'
                else:
                    op = 'w'

                # save the file
                out = open(filename, op)
                out.write(req.text)
                out.close()

                resp.status.result = Status.SUCCESS
        
        return resp

    '''
    add_type()
    Add a type of object to store in the Librarian workspace.
    '''
    def add_type(self, req):
        resp = AddTypeResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)

            # create a directory for objects of this type if necessary
            if not os.path.exists(path):
                os.mkdir(path)

            resp.status.result = Status.SUCCESS

        return resp

    '''
    load()
    Load the contents of a Librarian file as a string.
    '''
    def load(self, req):
        resp = LoadResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)
            filename = join(path, req.id)

            if not os.path.exists(path):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.NO_SUCH_TYPE
                resp.status.info = "Type %s does not exist!"%(req.type)
            elif not os.path.exists(filename):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.FILE_MISSING
                resp.status.info = "File %s does not exist as a member of type %s!"%(req.id, req.type)
            else:
                inf = open(filename, 'r')
                resp.text = inf.read()

                resp.status.result = Status.SUCCESS

        return resp

    '''
    load_params()
    Load onto the parameter server.
    '''
    def load_params(self, req):

        ''' EXAMPLE OF LOADING PARAMETERS FROM A YAML FILE:
        paramlist=rosparam.load_file("/path/to/myfile",default_namespace="my_namespace")
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)
        '''
        resp = LoadResponse()
        if len(req.type) == 0:
                resp.status.result = Status.FAILURE
                resp.status.error = Status.TYPE_MISSING
                resp.status.info = "No type provided!"
        else:
            path = join(self._root, req.type)
            filename = join(path, req.id)

            if not os.path.exists(path):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.NO_SUCH_TYPE
                resp.status.info = "Type %s does not exist!"%(req.type)
            elif not os.path.exists(filename):
                resp.status.result = Status.FAILURE
                resp.status.error = Status.FILE_MISSING
                resp.status.info = "File %s does not exist as a member of type %s!"%(req.id, req.type)
            else:
                paramlist=rosparam.load_file(filename)
                for params, ns in paramlist:
                    rosparam.upload_params(ns,params)

                resp.status.result = Status.SUCCESS

        return resp

    '''
    get_list()
    Returns the contents of a list as a list of strings.
    '''
    def get_list(self, req):
        resp = ListResponse()
        if len(req.type) == 0:
            path = self._root
        else:
            path = join(self._root, req.type)

        if not os.path.exists(path):
            resp.status.result = Status.FAILURE
            resp.status.error = Status.FILE_MISSING
            resp.status.info = "Could not find directory '%s'"%(path)
        else:
            resp.entries = os.listdir(path)
            resp.status.result = Status.SUCCESS
            resp.status.error = Status.NO_ERROR
        
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
