# -*- coding: utf-8 -*-
"""
    Alaa El Jawad
    ~~~~~~~~~~~~~
    Point class for manipulating 3D points
"""


class Point(object):
    """Point Object for manipulating 3D points"""

    def __init__(self, x, y, z):
        super(Point, self).__init__()
        self.x = x
        self.y = y
        self.z = z


def distance3D(p1, p2):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5
