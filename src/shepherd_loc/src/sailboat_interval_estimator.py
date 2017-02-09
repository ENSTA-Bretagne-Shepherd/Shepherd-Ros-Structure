#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
    Alaa El Jawad
    ~~~~~~~~~~~~~
    This node subscribes to the mesured position of the sailboat and
    publishes an interval where the sailboat should be
"""

import rospy
from shepherd_loc.msg import RosInterval, SailboatPoseInterval
from shepherd_disp.msg import SailboatPose

# --------------------------------------------------------------------------------
# ROS Node initialisation
# --------------------------------------------------------------------------------
rospy.init_node('sailboatX_locator')

# --------------------------------------------------------------------------------
# Get sensors precision from rosparam (if any)
# --------------------------------------------------------------------------------
# GPS Precision
gps_noise = 2
if rospy.has_param('sailboat_gps_noise'):
    gps_noise = rospy.get_param('sailboat_gps_noise')
    rospy.loginfo('I Precision was set to %f', gps_noise)
else:
    msg = 'GPS Precision was not set in param server, defaulting to: {} m'
    msg = msg.format(gps_noise)
    rospy.loginfo(msg)

# IMU Precision
imu_noise = 0.2
if rospy.has_param('sailboat_imu_noise'):
    imu_noise = rospy.get_param('sailboat_imu_noise')
    rospy.loginfo('IMU Precision was set to %f', imu_noise)
else:
    msg = 'IMU Precision was not set in param server, defaulting to: {} deg'
    msg = msg.format(imu_noise)
    rospy.loginfo(msg)

# --------------------------------------------------------------------------------
# Publisher of the interval of pose
# --------------------------------------------------------------------------------
pose_pub = rospy.Publisher('sailboat/pose_interval',
                           SailboatPoseInterval, queue_size=1)


# --------------------------------------------------------------------------------
# Subscribe to SailboatPose mesured data
# --------------------------------------------------------------------------------
def publish_pose_interval(msg):
    global gps_noise, imu_noise
    poseI = SailboatPoseInterval()
    poseI.x = RosInterval(msg.pose.x - gps_noise, msg.pose.x + gps_noise)
    poseI.y = RosInterval(msg.pose.y - gps_noise, msg.pose.y + gps_noise)
    poseI.theta = RosInterval(
        msg.pose.theta - imu_noise, msg.pose.theta + imu_noise)
    pose_pub.publish(poseI)


sb_pose_sub = rospy.Subscriber(
    'sailboat/pose_mes', SailboatPose, publish_pose_interval)

rospy.spin()
