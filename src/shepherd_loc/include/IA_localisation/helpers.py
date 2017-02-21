# -*- coding: utf-8 -*-
"""
    Alaa El Jawad
    ~~~~~~~~~~~~~
    Useful methods for localisation in the project
"""
from pyibex import *
from shepherd_msg.msg import RosInterval


def mypolarXY(x, y, r, alpha):
    myf1 = Function("x", "y", "(x-%f)^2 + (y-%f)^2" % (x, y))
    myf2 = Function("x", "y", "atan2(%f-y,%f-x)" % (y, x))
    # myf2 = Function("x", "y", "atan2(y-%f,x-%f)" % (y, x))
    myC1 = SepFwdBwd(myf1, r**2)
    myC2 = SepFwdBwd(myf2, alpha)
    ctcs = [myC1, myC2]
    # ctc = CtcQInter(ctcs, 0)
    sep = SepQInter(ctcs)
    sep.q = 0
    return sep


def distSep(x, y, r):
    myf1 = Function("X", "Y", "(X-%f)^2 + (Y-%f)^2" % (x, y))
    myC1 = SepFwdBwd(myf1, r**2)
    return myC1


def distSep3D(x, y, z, r):
    myf1 = Function(
        "X", "Y", "Z", "(X-[%f, %f])^2 + (Y-[%f, %f])^2 + (Z-[%f, %f])^2" % (
            x.lb(), x.ub(), y.lb(), y.ub(), z.lb(), z.ub()))
    myC1 = SepFwdBwd(myf1, r**2)
    return myC1


def distSep3Dtdoa(x, y, z, Z, c, t0):
    eq = "(X-[%f, %f])^2 + (Y-[%f, %f])^2 + ([%f, %f]-[%f, %f])^2 - %f*(t1-%f)"
    myf1 = Function("X", "Y", "t1",
                    eq % (x.lb(), x.ub(),
                          y.lb(), y.ub(),
                          Z.lb(), Z.ub(),
                          z.lb(), z.ub(), c, t0))
    myC1 = SepFwdBwd(myf1, Interval(0))
    return myC1


def fuse(intList):
    a = intList[0]
    for i in intList:
        a = a | i
    return a


class Point(object):
    """Point Object for manipulating 3D points"""

    def __init__(self, x, y, z):
        super(Point, self).__init__()
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other):
        return ((self.x - other.x)**2 +
                (self.y - other.y)**2 +
                (self.z - other.z)**2)**0.5


def distance3D(p1, p2):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5


class SailboatPoseHolderStamped(object):
    """docstring for SailboatPoseHolderStamped"""
    def __init__(self, x=0, y=0, dep=0, arr=0):
        super(SailboatPoseHolderStamped, self).__init__()
        self.x = x
        self.y = y
        self.dep = dep
        self.arr = arr
        self.dt = arr - dep


def ros2interval(msg):
    print '###', msg
    return Interval(msg.lb, msg.ub)


def interval2ros(I):
    return RosInterval(I.lb(), I.ub())
