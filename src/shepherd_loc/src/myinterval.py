# -*- coding: utf-8 -*-
"""
    Alaa El Jawad
    ~~~~~~~~~~~~~
    Useful methods that creates an abstraction of pyibex
"""
from vibes import *
from pyibex import *


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
    myf1 = Function(
        "X", "Y", "t1", "(X-[%f, %f])^2 + (Y-[%f, %f])^2 + ([%f, %f]-[%f, %f])^2 - %f*(t1-%f)" % (
            x.lb(), x.ub(), y.lb(), y.ub(), Z.lb(), Z.ub(), z.lb(), z.ub(), c, t0))
    myC1 = SepFwdBwd(myf1, Interval(0))
    return myC1


def fuse(intList):
    a=intList[0]
    for i in intList:
        a=a | i
    return a
