from pyibex import *
from helpers import distSep3Dtdoa, fuse, Point, distance3D, ros2interval


def localization(sailboats, depth, DT):
    # --------------------------------------------------------------------------------
    # Constant
    # --------------------------------------------------------------------------------
    # sound wave celerity into water
    c = 1500.

    # --------------------------------------------------------------------------------
    # Received data (noisy) --> Interval
    # --------------------------------------------------------------------------------
    sailboatsI = [Point(ros2interval(sb.x), ros2interval(sb.y), Interval(0))
                  for sb in sailboats.itervalues()]

    # information from the pressure sensor
    Z = ros2interval(depth)
    # Z = Interval(zmin, zmax).inflate(noisePressure)

    # --------------------------------------------------------------------------------
    # Separator
    # --------------------------------------------------------------------------------
    seps = []
    for s, dt in zip(sailboatsI, DT):
        # print 'Systeme:', s.x, s.y, s.z, Z, dt
        sep = distSep3Dtdoa(s.x, s.y, s.z, Z, c, dt * 1e-9, 0)
        seps.append(sep)

    sep = SepQInter(seps)
    sep.q = 0

    startI = IntervalVector(3)
    startI[0] = Interval(-200, 200)
    startI[1] = Interval(-200, 200)
    startI[2] = Interval(-200, 200)

    # SIVIA
    inside, outside, limit = pySIVIA(startI, sep, 5, draw_boxes=False)
    # print '>>>>>>>', limit
    box = fuse(inside + limit)
    buoyX, buoyY, buoyZ = box

    return buoyX, buoyY, Z
