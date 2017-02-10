#!/usr/bin/env python
import rospy
from models.sailboat import Sailboat
from shepherd_reg.msg import SailboatCmd
from shepherd_disp.msg import SailboatPose
from std_msgs.msg import Float64


def update_cmd(msg):
    global cmd
    print 'Updated cmd:', msg.rudder_angle, msg.sail_angle
    cmd = [msg.rudder_angle, msg.sail_angle]


def update_wind_direction(msg):
    global wind_direction
    wind_direction = msg.data


def update_wind_force(msg):
    global wind_force
    wind_force = msg.data


rospy.init_node('sailboat_simu')

sailboat = Sailboat(theta=0.1, v=3)
# Sailboat pose publisher
pose_pub = rospy.Publisher('sailboat/pose_real', SailboatPose, queue_size=1)

# Subscribe to the command of the sailboat
sub = rospy.Subscriber('sailboat/cmd', SailboatCmd, update_cmd)
# Subscribe to the wind
rospy.Subscriber('env/wind_direction', Float64, update_wind_direction)
rospy.Subscriber('env/wind_force', Float64, update_wind_force)


# Command
cmd = [0, 0]
wind_force = 3
wind_direction = 0

# rate
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    sailboat.simulate(cmd, wind_force, wind_direction)
    pose = SailboatPose()
    pose.pose.x = sailboat.x
    pose.pose.y = sailboat.y
    pose.pose.theta = sailboat.theta
    pose_pub.publish(pose)
    rate.sleep()
