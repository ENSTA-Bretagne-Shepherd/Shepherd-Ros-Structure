#!/usr/bin/env python
import rospy
from shepherd_disp.msg import SailboatPose
import matplotlib.pyplot as plt
import numpy as np


# --------------------------------------------------------------------------------
# Utilities
# --------------------------------------------------------------------------------


class SailboatPoseHolder(object):
    """docstring for SailboatPoseHolder"""
    MAX_HIST_SIZE = 500

    def __init__(self, pose):
        super(SailboatPoseHolder, self).__init__()
        self.pose = pose
        self.histX = []
        self.histY = []
        self.histT = []

    def add_new_pose(self, pose):
        self.pose = pose
        self.update_hist(pose.x, pose.y, pose.theta)

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


# --------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------
# Sailboat drawing
HULL = np.array([[-1, 5, 7, 7, 5, -1, -1, -1],
                 [- 2, -2, -1, 1, 2, 2, -2, -2],
                 [1, 1, 1, 1, 1, 1, 1, 1]])


def draw_sailboat(x, y, theta):
    # Rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta), x],
                  [np.sin(theta), np.cos(theta), y],
                  [0, 0, 1]])
    # Rotate
    hullr = np.dot(R, HULL)
    return hullr


def update_disp(msg, sailboat_name):
    global sailboats
    print 'Updating', sailboat_name
    sailboats[sailboat_name].add_new_pose(msg.pose)


def handle_close(event):
    global closed
    print 'Plot window closed !'
    closed = True


# Initialize node
rospy.init_node('display_simple')

# Subscriber to the sailboat position
rospy.Subscriber('sailboat1/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat1')
rospy.Subscriber('sailboat2/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat2')
rospy.Subscriber('sailboat3/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat3')
rospy.Subscriber('sailboat4/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat4')

# Sailboats pose
sailboats = {'sailboat1': SailboatPoseHolder(SailboatPose().pose),
             'sailboat2': SailboatPoseHolder(SailboatPose().pose),
             'sailboat3': SailboatPoseHolder(SailboatPose().pose),
             'sailboat4': SailboatPoseHolder(SailboatPose().pose)}

# Figure for display
fig = plt.figure("Display")
fig.canvas.mpl_connect('close_event', handle_close)
plt.show(block=False)

# Display rate
rate = rospy.Rate(10)

# Boolean for the state of the drawing window
closed = False

while not rospy.is_shutdown() and not closed:
    plt.clf()
    # Display axis
    minX, minY = 0, 0
    maxX, maxY = 0, 0
    for k in sailboats:
        sb = sailboats[k]
        plt.plot(sb.pose.x, sb.pose.y, 'ro')
        plt.plot(sb.histX, sb.histY, 'g')
        hull = draw_sailboat(sb.pose.x, sb.pose.y, sb.pose.theta)
        plt.plot(hull[0], hull[1], 'k', linewidth=2)
        # update axis
        maxX = max(maxX, sb.pose.x)
        maxY = max(maxY, sb.pose.y)
        minX = min(minX, sb.pose.x)
        minY = min(minY, sb.pose.y)
    plt.axis([minX - 150, maxX + 150, minY - 150, maxY + 150])
    plt.pause(rate.sleep_dur.to_sec())
