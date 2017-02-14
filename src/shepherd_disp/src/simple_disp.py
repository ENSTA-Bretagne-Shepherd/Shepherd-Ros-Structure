#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose
from shepherd_msg.msg import WorldInfo
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np


def draw_triangle():
    global cx, cy

    pt1x = cx + 50 * np.cos(1 * 2 * np.pi/3);
    pt1y = cy + 50 * np.sin(1 * 2 * np.pi/3);
    pt2x = cx + 50 * np.cos(2 * 2 * np.pi/3);
    pt2y = cy + 50 * np.sin(2 * 2 * np.pi/3);
    pt3x = cx + 50 * np.cos(3 * 2 * np.pi/3);
    pt3y = cy + 50 * np.sin(3 * 2 * np.pi/3);
    triangle = np.array([[pt1x, pt2x, pt3x, pt1x],
                         [pt1y, pt2y, pt3y, pt1y],
                         [   1,    1,    1,    1]])
    return triangle

def draw_sailboat():
    global x, y, theta
    # Original
    hull = np.array([[-1,  5,  7, 7, 5, -1, -1, -1],
                     [-2, -2, -1, 1, 2,  2, -2, -2],
                     [ 1,  1,  1, 1, 1,  1,  1,  1]])
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

def update_wind(msg):
    global wind_dir, wind_strength
    wind_dir, wind_strength = msg.wind_angle, msg.wind_strength

def update_center(msg):
    global cx, cy
    cx, cy = msg.data[0], msg.data[1]

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


def draw_buoy(xb=0,yb=0):
    angle = np.arange(0,3*np.pi,np.pi/10)
    buoy = [xb+np.cos(buoy),yb+np.sin(buoy)]
    return buoy

# Initialize node
rospy.init_node('display_simple')

# Subscriber to the sailboat position
rospy.Subscriber('sailboat/pose_real', SailboatPose, update_disp)
rospy.Subscriber('world/env', WorldInfo, update_wind)
rospy.Subscriber('sailboat/triangleCenter', Float64MultiArray, update_center)
rospy.Subscriber('buoy/pose_real',Point,update_disp) # Subcriber to buoy

# Data to display
x, y, theta = 0, 0, 0
wind_dir, wind_strength = 0, 0
cx, cy = 0, 0
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
    triangle = draw_triangle()
    plt.quiver(x+10,y+10, wind_strength*np.cos(wind_dir), wind_strength*np.sin(wind_dir))
    plt.plot(triangle[0], triangle[1], 'b', linewidth=1)
    plt.plot(hull[0], hull[1], 'k', linewidth=2)

    plt.axis([x - 150, x + 150, y - 150, y + 150])
    plt.pause(rate.sleep_dur.to_sec())
