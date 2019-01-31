#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('cmm_articulation')
import sys
import json
import pdb
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import time
from cmm_articulation.msg import ObjectPoses

def bb_poses_pub():
    pose_pub = rospy.Publisher('/bb_poses', ObjectPoses)
    poses_file_name = rospy.get_param('~poses_file_name', 'test.json')

    with open(poses_file_name) as f:
        data = json.load(f)

    # iterate through poses and publish to /bb_poses
    header = Header() # may want to include header information at some point...
    objs = data.keys()
    for t in range(len(data[objs[0]])):
        poses = []
        for obj in objs:
            obj_data = data[obj][t]
            pose = PoseStamped(header, Pose(Point(*obj_data[:3]), Quaternion(*obj_data[3:])))
            poses += [pose]
        pose_pub.publish(ObjectPoses(objs, poses))
        time.sleep(1)

def main():
    try:
        rospy.init_node('bb_poses_pub')
        bb_poses_pub()
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    main()
