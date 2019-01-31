#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('cmm_articulation')

from articulation_msgs.msg import ArticulatedObjectMsg

RIGID_PARAMS = ['rigid_position.x',
		'rigid_position.y',
		'rigid_position.z',
		'rigid_orientation.x',
		'rigid_orientation.y',
		'rigid_orientation.z',
		'rigid_orientation.w']
PRISMATIC_PARAMS = ['prismatic_dir.x',
		    'prismatic_dir.y',
		    'prismatic_dir.z',
		    'q_min[0]',
		    'q_max[0]'] + RIGID_PARAMS
REVOLUTE_PARAMS = ['rot_center.x',
		   'rot_center.y',
		   'rot_center.z',
		   'rot_axis.x',
		   'rot_axis.y',
		   'rot_axis.z',
		   'rot_axis.w',
		   'rot_orientation.x',
		   'rot_orientation.y',
		   'rot_orientation.z',
		   'rot_orientation.w',
		   'q_min[0]',
		   'q_max[0]',
		   'rot_radius']

def callback(data):
    print '---------------'
    # print dir(data)
    # print type(data.models)
    # print len(data.models)
    for model in data.models:
        print '====='
        # print dir(model)
        print model.name
        if model.name == 'rigid':
            params = [p for p in model.params if p.name in RIGID_PARAMS]
        elif model.name == 'prismatic':
            params = [p for p in model.params if p.name in PRISMATIC_PARAMS]
        elif model.name == 'rotational':
	    params = [p for p in model.params if p.name in REVOLUTE_PARAMS]
        for param in params:
            print param.name, param.value

if __name__ == '__main__':
    rospy.init_node('structure_saver')

    rospy.Subscriber('object', ArticulatedObjectMsg, callback)
    rospy.spin()
