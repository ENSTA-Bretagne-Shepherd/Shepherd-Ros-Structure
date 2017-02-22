#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose, Ping
# --------------------------------------------------------------------------------
# Subscribe to all sailboats
# --------------------------------------------------------------------------------

sb_pose = {'sailboat1': SailboatPose(),
		   'sailboat2': SailboatPose(),
		   'sailboat3': SailboatPose()}