#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose, PingVector, Ping, RosInterval
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


def extract_buoy_name_from_topic(topic):
    for s in topic.split('/'):
        if s.startswith('buoy'):
            return s


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

buoy_pubs = {}
for topic in buoy_poses.iterkeys():
    rospy.Subscriber(topic, Point, update_buoy_pose, callback_args=topic)
    b_name = extract_buoy_name_from_topic(topic)
    pub = rospy.Publisher(b_name + '/ping', PingVector, queue_size=1)
    buoy_pubs[topic] = pub

# --------------------------------------------------------------------------------
# Send a ping periodically
# --------------------------------------------------------------------------------

# Sound celerity (en m/s)
c_sound = 1500

# Period
ping_period = 2
if rospy.has_param('ping_interval'):
    ping_period = rospy.get_param('ping_interval')
    rospy.loginfo('Ping interval was set to %f', ping_period)
else:
    msg = 'Ping was not set in param server, defaulting to: {} seconds'
    msg = msg.format(ping_period)
    rospy.loginfo(msg)

rate = rospy.Rate(1. / ping_period)


def generatePing(sb_name, sb_pose, send_time, d_to_buoy):
    p = Ping()
    p.id = sb_name
    p.x = RosInterval()
    p.x.lb = sb_pose.pose.x - 0.3
    p.x.ub = sb_pose.pose.x + 0.3
    p.y = RosInterval()
    p.y.lb = sb_pose.pose.y - 0.3
    p.y.ub = sb_pose.pose.y + 0.3
    p.dep = rospy.Time.now()
    delay = d_to_buoy / c_sound
    p.arr = rospy.Time.from_sec(p.dep.to_sec() + delay)
    return p


def send_pings():
    global sb_pose, buoy_poses
    for b_topic, b_pose in buoy_poses.iteritems():
        b_name = b_topic.replace('/pose_real', '').replace('/', '')
        pings_for_buoy = PingVector()
        for name, pose in sb_pose.iteritems():
            dx = pose.pose.x - b_pose.x
            dy = pose.pose.y - b_pose.y
            # we suppose that sailboats are at 0 depth
            dz = b_pose.z
            d = sqrt(dx**2 + dy**2 + dz**2)
            # print '{} is at {} m from {}'.format(b_name, d, name)
            ping = generatePing(name, pose, rospy.Time.now(), d)
            pings_for_buoy.pings.append(ping)
        buoy_pubs[b_topic].publish(pings_for_buoy)
        msg ='pose bouee: {}, {}, {}'.format(b_pose.x, b_pose.y, b_pose.z)
        rospy.logwarn(msg)
        rospy.logdebug('-')
    rospy.logdebug('----')
    rospy.logwarn('ping sent')


while not rospy.is_shutdown():
    send_pings()
    rate.sleep()
