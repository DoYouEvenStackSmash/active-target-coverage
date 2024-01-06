#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
# from support.file_loader import *
from AnnotationLoader import AnnotationLoader as al
from StreamingObjectTrackManager import ObjectTrackManager as SOTM
import sys
import os
import json


def init_generic_object_tracker():
    """
    Initializer for a new object tracker
    """
    obj_tracker = SOTM()
    return obj_tracker


def import_tracks(an_json):  # , sys_path="."):
    """
    LOADER
    Wrapper function for loading a json file into a newly created ObjectTrackManager

    Returns an ObjectTrackManager
    """

    otm = SOTM()
    # tw = init_tracker_wrapper()
    # tw.obj_tracker = otm
    # otm.context_manager = tw

    otm.import_loco_fmt(an_json, sys.path)
    return otm


def fuse_annotations(infiles, outfile=None):
    """
    LOADER
    Loads annotations from a json file
    Imports and corrects annotations
    Serializes corrected annotations
    Does not return
    """
    trackers = []
    for infile in infiles:
        s = al.load_annotations_from_json_file(infile)
        # if not len(s["annotations"]):
        #   continue
        if len(s["annotations"]) < 7:
            continue
        trackers.append(import_tracks(s))

    super_otm = SOTM()

    # handle filenames
    fileset = {}
    sortkey = lambda x: x[1]
    for ot in trackers:
        for filename in ot.filenames:
            if filename not in fileset:
                fileset[filename] = len(fileset)

    # for i in ot.filenames:
    #   print("hao")

    # return
    super_otm.filenames = fileset  # [flist[i][0] for i in range(len(flist))]
    # return

    for ot in trackers:
        for old_track_id, track in ot.global_track_store.items():
            new_track_id = len(super_otm.global_track_store)
            track.track_id = new_track_id
            super_otm.global_track_store[new_track_id] = ot.get_track(old_track_id)

    super_otm.close_all_tracks()
    super_otm.link_all_tracks(2)
    e = super_otm.limited_export_loco_fmt()
    f = open("merged.json", "w")
    f.write(json.dumps(e, indent=2))
    f.close()


def main():
    infiles = sys.argv[1:]
    fuse_annotations(infiles)


if __name__ == "__main__":
    main()
