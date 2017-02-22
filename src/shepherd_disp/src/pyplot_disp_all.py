#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose
from shepherd_msg.msg import WorldInfo
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import numpy as np
import sea_plot_utility.sea_plot_api as seaplt


# --------------------------------------------------------------------------------
# Utilities
# --------------------------------------------------------------------------------


class PoseHolder(object):
    """docstring for SailboatPoseHolder"""
    MAX_HIST_SIZE = 500

    def __init__(self, pose):
        self.pose = pose
        self.histX = []
        self.histY = []
        self.histT = []

    def update_hist(self, x, y, theta):
        self.histX.append(x)
        self.histY.append(y)
        self.histT.append(theta)
        if len(self.histX) > SailboatPoseHolder.MAX_HIST_SIZE:
            del(self.histX[0])
        if len(self.histY) > SailboatPoseHolder.MAX_HIST_SIZE:
            del(self.histY[0])
        if len(self.histT) > SailboatPoseHolder.MAX_HIST_SIZE:
            del(self.histT[0])


class SailboatPoseHolder(PoseHolder):
    """docstring for SailboatPoseHolder"""
    MAX_HIST_SIZE = 500

    def __init__(self, pose):
        super(SailboatPoseHolder, self).__init__(pose)
        self.cx = 0
        self.cy = 0

    def updateTriCenter(self, cx, cy):
        self.cy = cy
        self.cx = cx

    def add_new_pose(self, pose):
        self.pose = pose
        self.update_hist(pose.x, pose.y, pose.theta)


class BuoyPoseHolder(PoseHolder):
    def __init__(self, pose):
        super(BuoyPoseHolder, self).__init__(pose)

    def add_new_pose(self, point):
        self.pose = point
        self.update_hist(point.x, point.y, point.z)


# --------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------
# Init entity numbers
buoysNb = 2
sailboatsNb = 4

def update_disp(msg, name):
    global sailboats, buoys
    # print 'Updating', sailboat_name
    # print('[INFO] updating : {}'.format(name))
    if name in sailboats:
        # print('[INFO] Adding new sailboat pose : {}'.format(name))
        sailboats[name].add_new_pose(msg.pose)
    else:
        # NOTE: verifier format msg
        # print('[INFO] Adding new buoy pose : {}'.format(name))
        buoys[name].add_new_pose(msg)


def update_wind(msg):
    global wind_dir, wind_strength
    wind_dir, wind_strength = msg.wind_angle, msg.wind_strength


def update_center(msg, name):
    global sailboats, buoys
    # print 'Updating center', sailboat_name
    if name in sailboats:
        sailboats[name].cx = msg.data[0]
        sailboats[name].cy = msg.data[1]
    else:
        pass
        # NOTE: verifier format msg
        # buoys[name].cx = msg.data[0]
        # buoys[name].cy = msg.data[1]


def handle_close(event):
    global closed
    print 'Plot window closed !'
    closed = True

# Initialize node
rospy.init_node('display_simple')

# BuoyPose
# Suscriber to the buoy position
# NOTE: verifier si Point est le bon message
buoys = dict()
for i in range(buoysNb):
    buoys['buoy{}'.format(i)] = BuoyPoseHolder(Point)
    rospy.Subscriber('buoy{}/pose_real'.format(i), Point,update_disp, callback_args='buoy{}'.format(i))
    # print('[INFO] Suscribed to {}'.format('buoy{}/pose_real'.format(i)))

rospy.Subscriber('world/env', WorldInfo, update_wind)

# Sailboats pose
# Subscriber to the sailboat position
# Suscriber to the center of the triangles
sailboats = dict()
for i in range(1, sailboatsNb+1):
    sailboats['sailboat{}'.format(i)] = SailboatPoseHolder(SailboatPose().pose)
    rospy.Subscriber('sailboat{}/pose_real'.format(i), SailboatPose, update_disp, callback_args='sailboat{}'.format(i))
    rospy.Subscriber('sailboat{}/triangleCenter'.format(i), Float64MultiArray, update_center, callback_args='sailboat{}'.format(i))
    # print('[INFO] Suscribed to {}'.format('sailboat{}/pose_real'.format(i)))

# Figure for display
fig = plt.figure("Display")
fig.canvas.mpl_connect('close_event', handle_close)
plt.show(block=False)

# Display rate
rate = rospy.Rate(10)

# Boolean for the state of the drawing window
closed = False

wind_dir, wind_strength = 0, 0

while not rospy.is_shutdown() and not closed:
    plt.clf()
    # Display axis
    minX, minY = 0, 0
    maxX, maxY = 0, 0

    for k in sailboats:
        sb = sailboats[k]

        plt.plot(sb.pose.x, sb.pose.y, 'ro')

        maxID = np.min([np.max([len(sb.histX),len(sb.histY)]),400])
        x = sb.histX[0:maxID]
        y = sb.histY[0:maxID]
        plt.plot(x, y, 'g')

        hull = seaplt.draw_sailboat(sb.pose.x, sb.pose.y, sb.pose.theta)
        triangleIn = seaplt.draw_triangle(sb.cx, sb.cy, 50-10*1.5)
        triangle = seaplt.draw_triangle(sb.cx, sb.cy, 50)
        triangleOut = seaplt.draw_triangle(sb.cx, sb.cy, 50+10*1.5)

        plt.plot(triangle[0], triangle[1], 'b', linewidth=1)
        plt.plot(triangleIn[0], triangleIn[1], 'r-', linewidth=1)
        plt.plot(triangleOut[0], triangleOut[1], 'r-', linewidth=1)
        plt.plot(hull[0], hull[1], 'k', linewidth=2)

        # update axis
        maxX = max(maxX, sb.pose.x)
        maxY = max(maxY, sb.pose.y)
        minX = min(minX, sb.pose.x)
        minY = min(minY, sb.pose.y)

    for bKey in buoys:
        print('[INFO] buoy name : {}'.format(bKey))
        # print('[INFO] buoy obj : {}'.format(buoys[bKey]))
        # print('[INFO] buoy pose.pose : {}'.format(buoys[bKey].pose))
        # print('[INFO] buoy x coord : {}'.format(buoys[bKey].pose.x))
        # print('[INFO] buoy y coord : {}'.format(buoys[bKey].pose.y))
        # print('[INFO] buoy y coord : {}'.format(buoys[bKey].pose.z))

        x = buoys[bKey].pose.x
        y = buoys[bKey].pose.y
        z = buoys[bKey].pose.z
        zmax = 50

        plt.plot(x, y, 'ko')
        buoy_shape = seaplt.draw_buoy_xy(x, y, z, zmax)
        plt.gcf().gca().add_artist(buoy_shape)

    plt.quiver(minX+10, minY+10, wind_strength*np.cos(wind_dir), wind_strength*np.sin(wind_dir))

    plt.axis([minX - 150, maxX + 150, minY - 150, maxY + 150])
    plt.axis('equal')
    plt.pause(rate.sleep_dur.to_sec())
