#!/usr/bin/env python

import roslib; roslib.load_manifest('cmm_articulation')
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
from articulation_models.transform_datatypes import *
from cmm_articulation.msg import ObjectPoses

import tf
import logging
import getopt
import colorsys
import numpy
import copy
import pdb

class articulation_busybox:
    def __init__(self):
        try:
            rospy.wait_for_service('fit_models', 5)
            rospy.wait_for_service('get_spanning_tree', 5)
            rospy.wait_for_service('get_fast_graph', 5)
            rospy.wait_for_service('visualize_graph', 5)
            print "services OK"
        except rospy.ROSException:
            print "Services not found"
            rospy.signal_shutdown('Quit')

        self.object_pub = rospy.Publisher('object', ArticulatedObjectMsg)
        rospy.Subscriber("/busybox/handle_poses", ObjectPoses, self.callback,queue_size=1)
        self.object_parts = []
        self.fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
        self.get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
        self.get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
        self.visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)

        self.sigma_position = rospy.get_param('~sigma_position',0.01)
        self.sigma_orientation = rospy.get_param('~sigma_orientation',0.3)
        self.reduce_dofs = rospy.get_param('~reduce_dofs',1)
        self.samples = rospy.get_param('~samples',50) #keep this number of samples

        self.samples = 50
        self.object_msg = ArticulatedObjectMsg()
        set_param(self.object_msg, "sigma_position", self.sigma_position, ParamMsg.PRIOR)
        set_param(self.object_msg, "sigma_orientation", self.sigma_orientation, ParamMsg.PRIOR)
        set_param(self.object_msg, "reduce_dofs", self.reduce_dofs, ParamMsg.PRIOR)
    
    def _hasnan(self, pose):
        if np.isnan(pose.position.x) or np.isnan(pose.position.y) or np.isnan(pose.position.z):
            return True
        return False

    def callback(self, objPoses):
        #print "adding pose.."
        transformedPoses = objPoses.poses
        objNames = objPoses.object_names

        if(len(self.object_parts)==0):
            self.object_parts += [TrackMsg(id=0,)]
            for (i, objName) in enumerate(objNames):
                self.object_parts += [TrackMsg(id=i+1,)]

        # reference frame
        identity = Pose(Point(0,0,0),Quaternion(0,0,0,1))
        # appends the new pose to the list of poses in the track message (try to keep all track messages the same length)
        self.object_parts[0].pose.append(identity)
        for (i, objName) in enumerate(objNames):
            if self._hasnan(transformedPoses[i].pose):
                self.object_parts[i+1].pose.append(self.object_parts[i+1].pose[-1])
            else:
                self.object_parts[i+1].pose.append(transformedPoses[i].pose)
        #print 'ITERATION: ', len(self.object_parts[1].pose)

        '''
        rospy.loginfo('sending tracks with '+('/'.join(["%d"%len(track.pose) for track in self.object_parts]))+' poses')
        for track in self.object_parts:
            track.header = transformedPoses.header
            self.pose_pub.publish(track)
            print 'publishing messages'
        '''
        # downsample or cut-off?
        self.object_msg.header = transformedPoses[0].header
        self.object_msg.parts = copy.deepcopy(self.object_parts)
        #if self.downsample:
        #    for part in self.object_msg.parts:
        #        if len(part.pose)>self.samples:
        #            part.pose = [p for (i,p) in enumerate(part.pose) if i % (len(part.pose) / self.samples + 1) == 0 or i==len(part.pose)-1]
        #else:
        for part in self.object_msg.parts:
            if len(part.pose)>self.samples:
                part.pose = part.pose[len(part.pose) - self.samples:]

        self.object_msg.header = transformedPoses[0].header

        request = ArticulatedObjectSrvRequest()
        request.object = self.object_msg

        parts = len(self.object_parts)
        # if len(self.object_parts[0].pose) % 10 == 0: return
        #print "calling fit service"
        response = self.fit_models(request)
        #print " fit service done"
        '''
        print '\n'.join(
            ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f sigma_pos=%f sigma_orient=%f'%(
            model.name[0:2],
            model.id/parts,
            model.id%parts,
            get_param(model,'bic'),
            get_param(model,'avg_error_position'),
            get_param(model,'avg_error_orientation'),
            get_param(model,'sigma_position'),
            get_param(model,'sigma_orientation')
            ) for model in response.object.models])
        '''
        #if len(response.object.models) > 0:
        #    print('models were found')
        request.object = copy.deepcopy(response.object)
        response = self.get_spanning_tree(request)

        self.object_pub.publish(response.object)
        self.object_msg.models = response.object.models

        # visualize_graph is for rviz
        '''
        request.object = copy.deepcopy(response.object)
        response = self.visualize_graph(request)

        rospy.loginfo('received articulation model: '+(' '.join(
        ['(%s:%d,%d)'%(model.name[0:2],model.id/parts,model.id%parts) for model in response.object.models])))

        print '\n'.join(
            ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f'%(
            model.name[0:2],
            model.id/parts,
            model.id%parts,
            get_param(model,'bic'),
            get_param(model,'avg_error_position'),
            get_param(model,'avg_error_orientation')
            ) for model in response.object.models])

        print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f"%(
            get_param(response.object,'dof'),
            get_param(response.object,'dof.nominal'),
            get_param(response.object,'bic'),
            get_param(response.object,'avg_error_position'),
            get_param(response.object,'avg_error_orientation')
            )
        '''
        #print "done evaluating new pose.."

def main():
    try:
        rospy.init_node('articulation_busybox')
        articulation_busybox()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
