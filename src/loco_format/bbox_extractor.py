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

    for image in images:
        imdict[image.fileName[:-3]] = []
    bbox_arr = [] * len(images)
    annos = top_loco_t.annotations
    for anno in annos:
        x,y,w,h = anno.bbox.x,anno.bbox.y,anno.bbox.w,anno.bbox.h
        imdict[anno.imageId[:-3]].append([x,y,w,h])
    wave_arr = []
    for key,wave in imdict.items():
        wave_arr.append([])
        for bbox in wave:
            wave_arr[-1].append((bbox[0],bbox[1]))
    return wave_arr

        
    # for k,v in imdict.items():
    #     print("{}".format(np.asarray(k[-4:-1]).astype(int)))
    #     for bbox in v:
    #         print("\t{}".format(bbox))
    # return imdict