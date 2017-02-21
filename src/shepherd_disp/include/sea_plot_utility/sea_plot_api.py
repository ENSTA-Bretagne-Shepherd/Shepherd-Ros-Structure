import numpy as np
import matplotlib.pyplot as plt

# Sailboat drawing
HULL = np.array([[-1,  5,  7, 7, 5, -1, -1, -1],
                 [-2, -2, -1, 1, 2,  2, -2, -2],
                 [ 1,  1,  1, 1, 1,  1,  1,  1]])

def draw_sailboat(x, y, theta):
    # Rotation matrix
    R = np.array([[np.cos(theta), -np.sin(theta), x],
                  [np.sin(theta), np.cos(theta), y],
                  [0, 0, 1]])
    # Rotate
    hullr = np.dot(R, HULL)
    return hullr

def draw_buoy_xy(xb,yb,zb,zmax):
    shape_buoy = plt.Circle((xb,yb),4,color=str(zb/zmax))
    return shape_buoy

def draw_buoy_xz(xb,yb,zb,ymax):
    shape_buoy = plt.Circle((xb,zb),4,color=str(yb/ymax))
    return shape_buoy

def draw_triangle(cx, cy, range):

    pt1x = cx + range * np.cos(1 * 2 * np.pi/3);
    pt1y = cy + range * np.sin(1 * 2 * np.pi/3);
    pt2x = cx + range * np.cos(2 * 2 * np.pi/3);
    pt2y = cy + range * np.sin(2 * 2 * np.pi/3);
    pt3x = cx + range * np.cos(3 * 2 * np.pi/3);
    pt3y = cy + range * np.sin(3 * 2 * np.pi/3);
    shape_triangle = np.array([[pt1x, pt2x, pt3x, pt1x],
                         [pt1y, pt2y, pt3y, pt1y],
                         [   1,    1,    1,    1]])
    return shape_triangle