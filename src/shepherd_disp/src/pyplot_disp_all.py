#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose
from shepherd_msg.msg import WorldInfo
from std_msgs.msg import Float64MultiArray
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
        self.cx = 0
        self.cy = 0

    def add_new_pose(self, pose):
        self.pose = pose
        self.update_hist(pose.x, pose.y, pose.theta)

    # def updateTriCenter(self, cx, cy):
    #     self.cy = cy
    #     self.cx = cx

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
HULL = np.array([[-1,  5,  7, 7, 5, -1, -1, -1],
                 [-2, -2, -1, 1, 2,  2, -2, -2],
                 [ 1,  1,  1, 1, 1,  1,  1,  1]])

def draw_triangle(cx, cy):

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
    # print 'Updating', sailboat_name
    sailboats[sailboat_name].add_new_pose(msg.pose)


def update_wind(msg):
    global wind_dir, wind_strength
    wind_dir, wind_strength = msg.wind_angle, msg.wind_strength


def update_center(msg, sailboat_name):
    global sailboats
    # print 'Updating center', sailboat_name
    sailboats[sailboat_name].cx = msg.data[0]
    sailboats[sailboat_name].cy = msg.data[1]


def handle_close(event):
    global closed
    print 'Plot window closed !'
    closed = True

# Initialize node
rospy.init_node('display_simple')


# Sailboats pose
sailboats = {'sailboat1': SailboatPoseHolder(SailboatPose().pose),
             'sailboat2': SailboatPoseHolder(SailboatPose().pose),
             'sailboat3': SailboatPoseHolder(SailboatPose().pose),
             'sailboat4': SailboatPoseHolder(SailboatPose().pose)}

# Subscriber to the sailboat position
rospy.Subscriber('sailboat1/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat1')
rospy.Subscriber('sailboat2/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat2')
rospy.Subscriber('sailboat3/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat3')
rospy.Subscriber('sailboat4/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat4')

rospy.Subscriber('world/env', WorldInfo, update_wind)
rospy.Subscriber('sailboat1/triangleCenter', Float64MultiArray, update_center, callback_args='sailboat1')
rospy.Subscriber('sailboat2/triangleCenter', Float64MultiArray, update_center, callback_args='sailboat2')
rospy.Subscriber('sailboat3/triangleCenter', Float64MultiArray, update_center, callback_args='sailboat3')
rospy.Subscriber('sailboat4/triangleCenter', Float64MultiArray, update_center, callback_args='sailboat4')

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

        maxID = np.max([len(sb.histX),len(sb.histY)])
        x = sb.histX[0:maxID]
        y = sb.histY[0:maxID]
        plt.plot(x, y, 'g')

        hull = draw_sailboat(sb.pose.x, sb.pose.y, sb.pose.theta)
        triangle = draw_triangle(sb.cx, sb.cy)
        if sb.cx == 0 and sb.cy == 0:
            print "ZEEEERROOOOOO : " + k

        plt.plot(triangle[0], triangle[1], 'b', linewidth=1)
        plt.plot(hull[0], hull[1], 'k', linewidth=2)

        # update axis
        maxX = max(maxX, sb.pose.x)
        maxY = max(maxY, sb.pose.y)
        minX = min(minX, sb.pose.x)
        minY = min(minY, sb.pose.y)

    plt.quiver(minX+10, minY+10, wind_strength*np.cos(wind_dir), wind_strength*np.sin(wind_dir))

    plt.axis([minX - 150, maxX + 150, minY - 150, maxY + 150])
    plt.pause(rate.sleep_dur.to_sec())
