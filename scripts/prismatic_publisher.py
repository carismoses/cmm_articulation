#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> .. 
"""

import roslib; roslib.load_manifest('cmm_articulation')
import rospy
from geometry_msgs.msg import PoseStamped
from articulation_models.track_utils import *

def main():
  rospy.init_node('simple_publisher')
  pub = rospy.Publisher('bb_poses', PoseStamped)
  
 

  loop = rospy.get_param("~loop",False)
  while True:    
    rospy.sleep(2)
    x = 0
    for i in range(1,30):
      x += 0.1
      msg = PoseStamped()
      msg.pose.position.x = x
      msg.pose.orientation.w = 1   
      pub.publish(msg)
      rospy.sleep(0.05)
      if rospy.is_shutdown():
        exit(0)
    if not loop:
       break

if __name__ == '__main__':
  main()

