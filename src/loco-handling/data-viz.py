#!/usr/bin/python3
import flatbuffers
import matplotlib.pyplot as plt
from LOCO.TopLoco import *
import sys

buf = open(sys.argv[1], "rb").read()
buf = bytearray(buf)
b = TopLoco.GetRootAsTopLoco(buf, 0)
top_loco_t = TopLocoT.InitFromObj(b)
linked_tracks = top_loco_t.linkedTracks
annos = top_loco_t.annotations
errors = []
for lt in linked_tracks:
    steps = lt.steps
    err = []
    for st in steps:
        err.append(annos[st].error)
    errors.append(err)
plt.plot(errors[0])
plt.show()

# print(c)
# print(b.Constants(2))
# c = TopLocoT.InitFromBuf(buf, 0)
# print(c.constants)
