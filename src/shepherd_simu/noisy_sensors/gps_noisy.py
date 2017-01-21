#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from shepherd_loc.msg import RosInterval, RosIntervalVector


def callback(msg):
    global x, y, z
    # Update x, y, z gps intervals
    X, Y, Z = msg.x, msg.y, msg.z
    x = RosInterval(lb=X - delta, ub=X + delta)
    y = RosInterval(lb=Y - delta, ub=Y + delta)
    z = RosInterval(lb=Z - delta, ub=Z + delta)
    print "GPS updated !"


# Node initialization
rospy.init_node('GPS_noisy')

# Subscribe to real gps data
sub = rospy.Subscriber('gps/real', Point, callback)
# Publish gps position with a noise in an interval
pub = rospy.Publisher('gps/noisy', RosIntervalVector, queue_size=1)

# Publish rate
rate = rospy.Rate(2)  # hz

# Noise
delta = 0.5

# Initialization of x, y, z interval values
X, Y, Z = 0, 0, 0
x = RosInterval(lb=X - delta, ub=X + delta)
y = RosInterval(lb=Y - delta, ub=Y + delta)
z = RosInterval(lb=Z - delta, ub=Z + delta)

gps = RosIntervalVector()
gps.vector = [x, y, z]

while not rospy.is_shutdown():
    # update if any modification happened in the callback
    gps.vector = [x, y, z]
    pub.publish(gps)
    rate.sleep()
