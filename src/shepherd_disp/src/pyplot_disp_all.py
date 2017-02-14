#!/usr/bin/env python
import rospy
from shepherd_msg.msg import SailboatPose
from shepherd_msg.msg import WorldInfo
from std_msgs.msg import Float64MultiArray
from geometry_msgs import Point
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

class BuoyPoseHolder(SailboatPoseHolder):

    def __init__(self,pose):
        super().__init__(pose)

    def add_new_pose(self,pose):
        super().add_new_pose(pose)

    def update_hist(self,x,y,theta):
        super().update_hist(x, y, theta)


# --------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------
# Sailboat drawing
HULL = np.array([[-1,  5,  7, 7, 5, -1, -1, -1],
                 [-2, -2, -1, 1, 2,  2, -2, -2],
                 [ 1,  1,  1, 1, 1,  1,  1,  1]])

def draw_triangle(cx, cy, range):

    pt1x = cx + range * np.cos(1 * 2 * np.pi/3);
    pt1y = cy + range * np.sin(1 * 2 * np.pi/3);
    pt2x = cx + range * np.cos(2 * 2 * np.pi/3);
    pt2y = cy + range * np.sin(2 * 2 * np.pi/3);
    pt3x = cx + range * np.cos(3 * 2 * np.pi/3);
    pt3y = cy + range * np.sin(3 * 2 * np.pi/3);
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

def draw_buoy(xb,yb):
    angle = np.arange(0,3*np.pi,np.pi/10)
    buoy = [xb+np.cos(buoy),yb+np.sin(buoy)]
    return buoy


def update_disp(msg, name):
    global sailboats, buoys
    # print 'Updating', sailboat_name
    if name in sailboats:
        sailboats[name].add_new_pose(msg.pose)
    else:
        # NOTE: verifier format msg
        buoys[name].add_new_pose(msg.pose)


def update_wind(msg):
    global wind_dir, wind_strength
    wind_dir, wind_strength = msg.wind_angle, msg.wind_strength


def update_center(msg, name):
    global sailboats, buoys
    # print 'Updating center', sailboat_name
    # if name in sailboats:
    sailboats[sailboat_name].cx = msg.data[0]
    sailboats[sailboat_name].cy = msg.data[1]
    # else:
    #     # NOTE: verifier format msg
    #     buoys[name].cx = msg.data[0]
    #     buoys[name].cy = msg.data[1]


def handle_close(event):
    global closed
    print 'Plot window closed !'
    closed = True

# Initialize node
rospy.init_node('display_simple')


# Sailboats pose
sailboats = {
            'sailboat1': SailboatPoseHolder(SailboatPose().pose),
            'sailboat2': SailboatPoseHolder(SailboatPose().pose),
            'sailboat3': SailboatPoseHolder(SailboatPose().pose),
            'sailboat4': SailboatPoseHolder(SailboatPose().pose)
            }
# NOTE: verifier si Point est le bon message
buoys = {
        'buoy0': BuoyPoseHolder(Point),
        'buoy1': BuoyPoseHolder(Point),
        'buoy1': BuoyPoseHolder(Point),
        'buoy3': BuoyPoseHolder(Point),
        'buoy4': BuoyPoseHolder(Point),
        'buoy5': BuoyPoseHolder(Point),
        'buoy6': BuoyPoseHolder(Point),
        'buoy7': BuoyPoseHolder(Point),
        'buoy8': BuoyPoseHolder(Point),
        'buoy9': BuoyPoseHolder(Point)
        }


# Subscriber to the sailboat position
rospy.Subscriber('sailboat1/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat1')
rospy.Subscriber('sailboat2/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat2')
rospy.Subscriber('sailboat3/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat3')
rospy.Subscriber('sailboat4/pose_real', SailboatPose,
                 update_disp, callback_args='sailboat4')

rospy.Subscriber('buoy0/pose_real', Point,update_disp, callback_args='buoy0')
rospy.Subscriber('buoy1/pose_real', Point,update_disp, callback_args='buoy1')
rospy.Subscriber('buoy2/pose_real', Point,update_disp, callback_args='buoy2')
rospy.Subscriber('buoy3/pose_real', Point,update_disp, callback_args='buoy3')
rospy.Subscriber('buoy4/pose_real', Point,update_disp, callback_args='buoy4')
rospy.Subscriber('buoy5/pose_real', Point,update_disp, callback_args='buoy5')
rospy.Subscriber('buoy6/pose_real', Point,update_disp, callback_args='buoy6')
rospy.Subscriber('buoy7/pose_real', Point,update_disp, callback_args='buoy7')
rospy.Subscriber('buoy8/pose_real', Point,update_disp, callback_args='buoy8')
rospy.Subscriber('buoy9/pose_real', Point,update_disp, callback_args='buoy9')

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

        maxID = np.min([np.max([len(sb.histX),len(sb.histY)]),400])
        x = sb.histX[0:maxID]
        y = sb.histY[0:maxID]
        plt.plot(x, y, 'g')

        hull = draw_sailboat(sb.pose.x, sb.pose.y, sb.pose.theta)
        triangleIn = draw_triangle(sb.cx, sb.cy, 50-10*1.5)
        triangle = draw_triangle(sb.cx, sb.cy, 50)
        triangleOut = draw_triangle(sb.cx, sb.cy, 50+10*1.5)
        if sb.cx == 0 and sb.cy == 0:
            print "ZEEEERROOOOOO : " + k

        plt.plot(triangle[0], triangle[1], 'b', linewidth=1)
        plt.plot(triangleIn[0], triangleIn[1], 'r-', linewidth=1)
        plt.plot(triangleOut[0], triangleOut[1], 'r-', linewidth=1)
        plt.plot(hull[0], hull[1], 'k', linewidth=2)

        # update axis
        maxX = max(maxX, sb.pose.x)
        maxY = max(maxY, sb.pose.y)
        minX = min(minX, sb.pose.x)
        minY = min(minY, sb.pose.y)

    for by in buoys:

        plt.plot(by.pose.x, by.pose.y, 'ko')
        buoy = draw_buoy(by.pose.x, by.pose.y)
        plt.fill(buoy[0], buoy[1], 'r',)





    plt.quiver(minX+10, minY+10, wind_strength*np.cos(wind_dir), wind_strength*np.sin(wind_dir))

    plt.axis([minX - 150, maxX + 150, minY - 150, maxY + 150])
    plt.axis('equal')
    plt.pause(rate.sleep_dur.to_sec())
