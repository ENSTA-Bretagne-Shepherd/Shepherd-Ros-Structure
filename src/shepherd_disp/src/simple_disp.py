#!/usr/bin/env python
import rospy
from shepherd_disp.msg import SailboatPose
import matplotlib.pyplot as plt
import numpy as np


def draw_sailboat():
    global x, y, theta
    # Original
    hull = np.array([[-1, 5, 7, 7, 5, -1, -1, -1],
                     [- 2, -2, -1, 1, 2, 2, -2, -2],
                     [1, 1, 1, 1, 1, 1, 1, 1]])
    # Rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta), x],
                  [np.sin(theta), np.cos(theta), y],
                  [0, 0, 1]])
    # Rotate
    hull = np.dot(R, hull)
    return hull


def update_disp(msg):
    global x, y, theta
    x, y, theta = msg.pose.x, msg.pose.y, msg.pose.theta


def handle_close(event):
    global closed
    print 'Plot window closed !'
    closed = True


def update_trace():
    global xt, yt, thetat, x, y, theta
    MAX_SIZE = 500
    xt.append(x)
    yt.append(y)
    thetat.append(theta)
    if len(xt) > MAX_SIZE:
        del(xt[0])
    if len(yt) > MAX_SIZE:
        del(yt[0])
    if len(thetat) > MAX_SIZE:
        del(thetat[0])


# Initialize node
rospy.init_node('display_simple')

# Subscriber to the sailboat position
rospy.Subscriber('sailboat/pose_real', SailboatPose, update_disp)

# Data to display
x, y, theta = 0, 0, 0
# trace
xt, yt, thetat = [], [], []

# Figure for display
fig = plt.figure("Display")
fig.canvas.mpl_connect('close_event', handle_close)
plt.show(block=False)

# Display rate
rate = rospy.Rate(10)

# Boolean for the state of the drawing window
closed = False

while not rospy.is_shutdown() and not closed:
    update_trace()
    plt.clf()
    plt.plot(x, y, 'ro')
    plt.plot(xt, yt, 'g')
    hull = draw_sailboat()
    plt.plot(hull[0], hull[1], 'k', linewidth=2)
    plt.axis([x - 150, x + 150, y - 150, y + 150])
    plt.pause(rate.sleep_dur.to_sec())
