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

fig, axs = plt.subplots(len(errors))
if len(errors) > 1:
    for eidx, err in enumerate(errors):
        axs[eidx].plot(errors[eidx])
else:
    axs.plot(errors[0])
plt.show()

# print(c)
# print(b.Constants(2))
# c = TopLocoT.InitFromBuf(buf, 0)
# print(c.constants)
