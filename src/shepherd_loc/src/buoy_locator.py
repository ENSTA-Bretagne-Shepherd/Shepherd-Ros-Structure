#!/usr/bin/env python

import rospy
from shepherd_msg.msg import BuoyPoseInterval, DepthInterval, PingVector
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from IA_localisation.helpers import SailboatPoseHolderStamped, interval2ros
from IA_localisation.tdoa import localization


# --------------------------------------------------------------------------------
# Subscribers callbacks
# --------------------------------------------------------------------------------

# @profile
def locate_buoy(msg):
    """
    This is called when we receive the array of pings
    """
    global sailboats, depth, loci_pub, loce_pub
    if len(msg.pings) != 4:
        rospy.logwarn("didn't receive 4 pings but only: %d", len(msg.pings))
        return
    rospy.logwarn('received a ping')
    # First update all sailboats position from each ping
    for ping in msg.pings:
        if ping.id in sailboats:
            sailboats[ping.id] = SailboatPoseHolderStamped(ping.x, ping.y, ping.dep.to_nsec(), ping.arr.to_nsec())
        else:
            rospy.logwarn('%s not in sailboats', ping.id)
            return
    # Then use tdoa to localize the sailboats
    dts = [sb.dt for sb in sailboats.itervalues()]
    dts = [dt - min(dts) for dt in dts]
    # msg = 'TABLEAU DTS : {}'.format(dts)
    # rospy.loginfo(msg)
    rospy.logwarn('Starting localization with:')
    print (sailboats, depth, dts)
    x, y, z = localization(sailboats, depth, dts)
    msg = 'Bouee localise a {}, {}, {}'.format(x, y, z)
    rospy.logwarn(msg)
    rospy.logdebug(msg)
    # And publish it
    b_pose = BuoyPoseInterval()
    b_pose.x = interval2ros(x)
    b_pose.y = interval2ros(y)
    b_pose.z = interval2ros(z)
    loci_pub.publish(b_pose)
    point = Point()
    point.x = (x.ub() + x.lb()) / 2
    point.y = (y.ub() + y.lb()) / 2
    point.z = (z.ub() + z.lb()) / 2
    rospy.loginfo('Bouee estimee a %f, %f, %f', point.x, point.y, point.z)
    loce_pub.publish(point)


def udpate_depth(msg):
    global depth, depth_sensor_noise
    depth_est = DepthInterval()
    depth_est.depth.lb = msg.data - depth_sensor_noise
    depth_est.depth.ub = msg.data + depth_sensor_noise
    depth = depth_est.depth


# --------------------------------------------------------------------------------
# Node initialization
# --------------------------------------------------------------------------------
rospy.init_node('buoy_localization')

# --------------------------------------------------------------------------------
# Global holders
# --------------------------------------------------------------------------------


sailboats = {'sailboat1': SailboatPoseHolderStamped(),
             'sailboat2': SailboatPoseHolderStamped(),
             'sailboat3': SailboatPoseHolderStamped(),
             'sailboat4': SailboatPoseHolderStamped()}

# Depth interval received by the simulation
depth = DepthInterval().depth

# --------------------------------------------------------------------------------
# Retrieve data about the depth noise sensor
# --------------------------------------------------------------------------------

depth_sensor_noise = 0.2
if rospy.has_param('/buoy_depth_sensor_noise'):
    depth_sensor_noise = rospy.get_param('/buoy_depth_sensor_noise')
    rospy.loginfo('Depth Precision was set to %f', depth_sensor_noise)
else:
    msg = 'Depth Precision was not set in param server, defaulting to: {} meters'
    msg = msg.format(depth_sensor_noise)
    rospy.loginfo(msg)


# --------------------------------------------------------------------------------
# Subscriber to the ping topic
# --------------------------------------------------------------------------------
ping_sub = rospy.Subscriber('ping', PingVector, locate_buoy)

# --------------------------------------------------------------------------------
# Subscriber to the depth sensor topic
# --------------------------------------------------------------------------------
depth_sub = rospy.Subscriber('depth_noisy', Float64, udpate_depth)

# --------------------------------------------------------------------------------
# Publisher of the buoy position
# --------------------------------------------------------------------------------
loci_pub = rospy.Publisher('pose_interval', BuoyPoseInterval, queue_size=1)
loce_pub = rospy.Publisher('pose_est', Point, queue_size=1)

# --------------------------------------------------------------------------------
# Spin the node
# --------------------------------------------------------------------------------

rospy.spin()
