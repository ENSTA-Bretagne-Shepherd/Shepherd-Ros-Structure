#!/usr/bin/env python

import rospy
from shepherd.msg import Msg_Simu2Loc
from shepherd.msg import Msg_Loc2Simu
from shepherd.msg import Msg_pressure
from shepherd_localization_TDOA import *
from pyibex import *

def callback_S2L(msg):
	Id = int(msg.id[-1])
	xmin = msg.xmin
	xmax = msg.xmax
	ymin = msg.ymin
	ymax = msg.ymax
	hourarr = msg.hourarr
	minarr = msg.minarr
	secarr = msg.secarr
	milarr = msg.milarr
	hourdep = msg.hourdep
	mindep = msg.mindep
	secdep = msg.secdep
	mildep = msg.mildep
	sailboats[Id] = [Interval(xmin,xmax),Interval(ymin,ymax),hourarr,minarr,secarr,milarr]
	DT[Id] = milarr-mildep + (secarr-secdep)*60 + (minarr-mindep)*3600 + (hourarr-hourdep)*3600*60 
	voilier[Id] = 1
	if(voilier[0]==1 and voilier[1]==1 and voilier[2]==1 and voilier[3]==1):
		DT = dtArrivee(DT)
		x, y, z = localization(sailboats, zmin, zmax, hourdep, mindep, secdep, mildep, DT)
		msg = Msg_Loc2Simu()
		msg.id = 2
		msg.xmin = x.lb()
		msg.xmax = x.ub()
		msg.ymin = y.lb()
		msg.ymax = y.ub()
		msg.zmin = zmin
		msg.zmax = zmax
		pub.publish(msg)
		voilier = [0,0,0,0]


def callback_z(msg):
	zmin = msg.zmin
	zmax = msg.zmax

def dtArrivee(DT):
	mini = min(DT)
	for i in range(3):
		DT[i] = DT[i]-mini
	return DT


sailboats = [[Interval(0),Interval(0),0,0,0,0],[Interval(0),Interval(0),0,0,0,0],[Interval(0),Interval(0),0,0,0,0],[Interval(0),Interval(0),0,0,0,0]]
voilier = [0,0,0,0]
DT = [0,0,0,0]
hourdep, mindep, secdep, mildep = 0, 0, 0, 0
rospy.init_node('loc')
sub_ping = rospy.Subscriber('simu2loc', Msg_Simu2Loc, callback_S2L)
sub_pressure = rospy.Subscriber('pressure', Msg_pressure, callback_z)
pub = rospy.Publisher('loc2simu', Msg_Loc2Simu)

	
rospy.spin()
