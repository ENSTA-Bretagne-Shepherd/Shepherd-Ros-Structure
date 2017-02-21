from pyibex import *
from helpers import distSep3Dtdoa, fuse, Point, distance3D, ros2interval


def localization(sailboats, depth, DT):
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
    # noise = 0.5
    # noiseGPS = 0.2
    # noiseTime = 0.5
    # noisePressure = 1

    # sailboatsI = [Point(Interval(sb.x).inflate(noiseGPS),
    #                   Interval(sb.y).inflate(noiseGPS),
    #                   Interval(sb.z).inflate(noiseGPS)) for sb in Sailboats]
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
        sep = distSep3Dtdoa(s.x, s.y, s.z, Z, c, dt.to_sec())
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
