#!/usr/bin/env python

import rospy
from shepherd_msg.msg import BuoyPoseInterval, DepthInterval, PingVector
from std_msgs.msg import Float64
from IA_localisation.helpers import SailboatPoseHolderStamped, interval2ros
from IA_localisation.tdoa import localization


# --------------------------------------------------------------------------------
# Subscribers callbacks
# --------------------------------------------------------------------------------

def locate_buoy(msg):
    """
    This is called when we receive the array of pings
    """
    global sailboats, depth, loc_pub
    if len(msg.pings) != 4:
        rospy.logwarn("didn't receive 4 pings but only: %d", len(msg.pings))
        return
    # First update all sailboats position from each ping
    for ping in msg.pings:
        if ping.id in sailboats:
            sailboats[ping.id] = SailboatPoseHolderStamped(
                ping.x, ping.y, ping.dep, ping.arr)
        else:
            rospy.logwarn('%s not in sailboats', ping.id)
            return
    # Then use tdoa to localize the sailboats
    dts = [sb.dt for sb in sailboats.itervalues()]
    dts = [dt - min(dts) for dt in dts]
    print '>>>', depth
    x, y, z = localization(sailboats, depth, dts)
    # And publish it
    b_pose = BuoyPoseInterval()
    b_pose.x = interval2ros(x)
    b_pose.y = interval2ros(y)
    b_pose.z = interval2ros(z)
    loc_pub.publish(b_pose)


def udpate_depth(msg):
    global depth, depth_sensor_noise
    depth_est = DepthInterval()
    depth_est.depth.lb= msg.data-depth_sensor_noise
    depth_est.depth.ub = msg.data+depth_sensor_noise
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
if rospy.has_param('buoy_depth_sensor_noise'):
    depth_sensor_noise = rospy.get_param('buoy_depth_sensor_noise')
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
loc_pub = rospy.Publisher('pose_est', BuoyPoseInterval, queue_size=1)

# --------------------------------------------------------------------------------
# Spin the node
# --------------------------------------------------------------------------------

rospy.spin()
