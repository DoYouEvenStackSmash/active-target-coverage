#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *


origin = Position(100,100, 0)

sa = init_sensing_agent(origin=origin)
zx,zy,zz = sa.get_center().get_cartesian_coordinates()

# prints a sensor coordinate to the left of center line
pxny = Position(zx + 10, zy - 5, zz)
det_pxny = sa.transform_to_local_detection_coord(pxny)
print(det_pxny)
print(sa.transform_to_local_sensor_coord((0,0,0), det_pxny))

# prints a sensor coordinate on center line
px0y = (zx + 10, zy, zz)
det_px0y = sa.transform_to_local_detection_coord(px0y)
print(sa.transform_to_local_sensor_coord((0,0,0), det_px0y))

# prints a sensor coordinate to the right of center line
pxpy = (zx + 10, zy + 5, zz)
det_pxpy = sa.transform_to_local_detection_coord(pxpy)
print(sa.transform_to_local_sensor_coord((0,0,0), det_pxpy))
