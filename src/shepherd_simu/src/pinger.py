#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose, Ping
from geometry_msgs.msg import Point
from math import sqrt


# --------------------------------------------------------------------------------
# Node initialisation
# --------------------------------------------------------------------------------
rospy.init_node('pinger')


# --------------------------------------------------------------------------------
# Subscribe to all sailboats' real poses
# --------------------------------------------------------------------------------
def update_sb_pose(msg, name):
    """
    Defines what to do when we receive a new sailboat pose
    """
    global sb_pose
    sb_pose[name] = msg


sb_pose = {'sailboat1': SailboatPose(),
           'sailboat2': SailboatPose(),
           'sailboat3': SailboatPose(),
           'sailboat4': SailboatPose()}


for sb in sb_pose.iterkeys():
    rospy.Subscriber(sb + '/pose_real', SailboatPose,
                     update_sb_pose, callback_args=sb)


# --------------------------------------------------------------------------------
# Subscribe to all buoys' real poses
# --------------------------------------------------------------------------------
def update_buoy_pose(msg, topic):
    """
    Defines what to do when we receive a new sailboat pose
    """
    global buoy_poses
    buoy_poses[topic] = msg


def get_all_published_buoy_topics():
    """
    get all published buoy topics
    WARNING
    !!! this must be launched after all buoy nodes have been initialized
    All launched nodes afterwards won't be detected
    """

    topics = rospy.get_published_topics()
    # print topics
    buoy_poses = {}
    for topic, msg_type in topics:
        if (msg_type == 'geometry_msgs/Point' and
                all(kw in topic.lower() for kw in ['buoy', 'real'])):
            rospy.loginfo('%s with type: %s was found', topic, msg_type)
            buoy_poses[topic] = Point()

    return buoy_poses


buoy_poses = get_all_published_buoy_topics()

for topic in buoy_poses.iterkeys():
    rospy.Subscriber(topic, Point, update_buoy_pose, callback_args=topic)


# --------------------------------------------------------------------------------
# Send a ping periodically
# --------------------------------------------------------------------------------

# Publisher
# ping_pub = rospy.Publisher('')

# Period
ping_period = 10
if rospy.has_param('ping_period'):
    ping_period = rospy.get_param('ping_period')
    rospy.loginfo('Ping period was set to %f', ping_period)
else:
    msg = 'Ping was not set in param server, defaulting to: {} seconds'
    msg = msg.format(ping_period)
    rospy.loginfo(msg)

rate = rospy.Rate(1. / ping_period)


def send_pings():
    global sb_pose, buoy_poses
    for b_topic, b_pose in buoy_poses.iteritems():
        b_name = b_topic.replace('/pose_real', '').replace('/', '')
        for name, pose in sb_pose.iteritems():
            dx = pose.pose.x - b_pose.x
            dy = pose.pose.y - b_pose.y
            # we suppose that sailboats are at 0 depth
            dz = b_pose.z
            d = sqrt(dx**2 + dy**2 + dz**2)
            print '{} is at {} m from {}'.format(b_name, d, name)
        print '-'
    print '----'


while not rospy.is_shutdown():
    send_pings()
    rate.sleep()
