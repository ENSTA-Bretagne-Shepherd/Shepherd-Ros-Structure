from pyibex import *
from vibes import *
from myinterval import distSep3Dtdoa, fuse
from point import *


def localization(sailboats, zmin, zmax, hourdep, mindep, secdep, mildep, DT):
	# --------------------------------------------------------------------------------
	# Real data and positions
	# --------------------------------------------------------------------------------
	# sound wave celerity into water
	c = 1500

	# sailboats => sailboat are all on the water surface

	# t => Time message sent

	# --------------------------------------------------------------------------------
	# Received data (noisy) --> Interval
	# --------------------------------------------------------------------------------
	# noise
	noise = 0.5
	noiseGPS = 0.2
	noiseTime = 0.5
	noisePressure = 1

	#sailboatsI = [Point(Interval(sb.x).inflate(noiseGPS),
	#	                Interval(sb.y).inflate(noiseGPS),
	#	                Interval(sb.z).inflate(noiseGPS)) for sb in Sailboats]

	sailboatsI = [Point(sailboats[i][0],sailboats[i][1],Interval(0)) for i in range(4)]

	# Buoy position
	buoyR = Point(0, 0, 0)
	# real distance to the sailboats
	DR = [distance3D(buoyR, sb) for sb in sailboatsI]

	D = [Interval(dr).inflate(noise) for dr in DR]

	t0 = 0

	# information from the pressure sensor
	Z = Interval(zmin,zmax).inflate(noisePressure)


	# --------------------------------------------------------------------------------
	# Separator
	# --------------------------------------------------------------------------------
	seps = []
	for s,dt in zip(sailboatsI,DT):
		sep = distSep3Dtdoa(s.x, s.y, s.z, Z, c, dt)
		seps.append(sep)

	sep = SepQInter(seps)
	sep.q = 0

	startI = IntervalVector(3)
	startI[0] = Interval(-20, 20)
	startI[1] = Interval(-20, 20)
	startI[2] = Interval(-20, 20)

	# SIVIA
	inside, outside, limit = pySIVIA(startI, sep, 1, draw_boxes=False)
	box = fuse(inside + limit)
	buoyX, buoyY, buoyZ = box

	return buoyX, buoyY, buoyZ
