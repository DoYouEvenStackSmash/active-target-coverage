#!/usr/bin/python3
import sys

sys.path.append(".")
import flatbuffers
import matplotlib.pyplot as plt
from LOCO.TopLoco import *


def load_waves(filename):
    buf = open(filename, "rb").read()
    buf = bytearray(buf)
    # get toploco
    b = TopLoco.GetRootAsTopLoco(buf, 0)
    top_loco_t = TopLocoT.InitFromObj(b)
    # get linked tracks
    linked_tracks = top_loco_t.linkedTracks
    images = top_loco_t.images
    imdict = {}
    suffix = np.array2string(np.array(images[0].fileName)).split(".")[-1][:-1]
    # print(suffix)
    imlist = []
    for image in images:
        fname = np.array2string(np.array(image.fileName).astype(str))
        # print(fname)
        fname = str(fname[1:-1])
        if fname not in imdict:
            imlist.append(fname)
            imdict[fname] = []
    # bbox_arr = [] * len(images)
    annos = top_loco_t.annotations
    for anno in annos:
        x, y, w, h = anno.bbox.x, anno.bbox.y, anno.bbox.w, anno.bbox.h
        fname = imlist[anno.imageId]
        # print(fname)
        imdict[fname].append([x, y, w, h])
    return imdict
    # wave_arr = []
    # for key,wave in imdict.items():
    #     wave_arr.append(wave)

    #     # for bbox in wave:
    #     #     wave_arr[-1].append(bbox)
    # return wave_arr

    # for k,v in imdict.items():
    #     print("{}".format(np.asarray(k[-4:-1]).astype(int)))
    #     for bbox in v:
    #         print("\t{}".format(bbox))
    # return imdict


def convert_back_to_world_coordinates(filename):
    buf = open(filename, "rb").read()
    buf = bytearray(buf)
    # get toploco
    b = TopLoco.GetRootAsTopLoco(buf, 0)
    top_loco_t = TopLocoT.InitFromObj(b)
    # get linked tracks
    linked_tracks = top_loco_t.linkedTracks
    annos = top_loco_t.annotations
    fov_width = top_loco_t.sensorParams
    global_angle = lambda x, fov_width, orientation: adjust_angle(
        x / 100 * fov_width - fov_width / 2 + orientation
    )
    global_posn = lambda origin, r, agent_angle: mfn.pol2car(origin, x, agent_angle)
    states = top_loco_t.states
    trkmap = {}
    for anno in annos:
        trackmapIndex = annno.trackmapIndex
        if trackmapIndex not in trkmap:
            trkmap[trackmapIndex] = []

        state = states[anno.stateId]
        x, y, w, h = anno.bbox.x, anno.bbox.y, anno.bbox.w, anno.bbox.h
        origin = state.position
        orientation = state.orientation
        agent_angle = global_angle(x, fov_width, orientation)
        target_posn = global_posn(origin, y, agent_angle)
        trkmap[trackmapIndex].append({"x": target_posn[0], "y": target_posn[1]})
    for t in trkmap:
        f = open(f"track_{str(t)}.json", "w")
        f.write(json.dumps({"points": trkmap[t]}, indent=2))
        f.close()
