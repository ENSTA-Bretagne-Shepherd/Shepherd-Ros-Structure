#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from shepherd_loc.msg import RosInterval

#  -------------------------------
# |===============================|
# |==========SUBCRIBER============|
# |===============================|


def callback(msg):
    global imu
    imu = msg.data
    print("SUB %f" % msg.data)


rospy.init_node("imu_noisy")

sub = rospy.Subscriber("imu/real", Float32, callback)


#  -------------------------------
# |===============================|
# |==========PUBLISHER============|
# |===============================|

pub = rospy.Publisher("imu/noisy", RosInterval, queue_size=1)
rate = rospy.Rate(2)

imu = 5
delta = 0.5

msg = RosInterval(lb=imu - delta, ub=imu + delta)

while not rospy.is_shutdown():
    pub.publish(msg)
