#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('cmm_articulation')
import json

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

class StructureSub:
	def __init__(self):
		self.models = {}

	def callback(self, data):
		for model in data.models:
			#print '====='
			#print model.name
			if model.name == 'rigid':
				params = [p for p in model.params if p.name in RIGID_PARAMS]
			elif model.name == 'prismatic':
				params = [p for p in model.params if p.name in PRISMATIC_PARAMS]
			elif model.name == 'rotational':
				params = [p for p in model.params if p.name in REVOLUTE_PARAMS]

			self.models[model.name] = {}
			for param in params:
				self.models[model.name][param.name] = param.value

		print('=== in self.models to json')
		for model in self.models.keys():
			print(model)
			#print(self.models[model_id]['type'])

		file_name = rospy.get_param('~structure_file_name', 'structure.json')
		with open(file_name, 'w') as handle:
			json.dump(self.models, handle)

if __name__ == '__main__':
	rospy.init_node('structure_saver')
	struct_sub = StructureSub()
	rospy.Subscriber('object', ArticulatedObjectMsg, struct_sub.callback)
	rospy.spin()
