#!/usr/bin/python3
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn

# from Agent import *

from PIL import Image, ImageDraw
import collections

# from aux_functions import *

# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from Target import Target

import pygame
import numpy as np
import sys
import time
import json

COLLISION_THRESHOLD = 10
VERBOSE = True
SAMPLE_RATE = 400
LALT = 256
LSHIFT = 1
LCTRL = 64
SPACE = 32
OFFT = 20
SPLINE_COUNT = 2
TRANSLATE = False


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


def import_agent_record(screen, agent_record):
    """
    Imports a LOCO formatted json

    Returns a json of some sort
    """

    fov_width = agent_record["sensor_params"]["fov_width"]
    fov_radius = agent_record["sensor_params"]["fov_radius"]

    trackmap = agent_record["trackmap"]
    lt = agent_record["linked_tracks"]

    get_center = lambda state: state["position"]
    get_orientation = lambda state: state["orientation"]
    annotations = agent_record["annotations"]
    states = agent_record["states"]
    # for anno in annotations:

    for track in lt:
        pts = []
        color = None
        for step_id in track["steps"]:
            anno = annotations[step_id]
            color = anno["track_color"]
            state = states[anno["state_id"]]
            x, y, w, h = anno["bbox"]
            theta = (x - 50) / Sensor.WINDOW_WIDTH * fov_width
            theta = adjust_angle(get_orientation(state) + theta)
            r = y
            pt = mfn.pol2car(get_center(state), r, theta)
            pts.append[pt]

        render_path(screen, pts, color)
    pygame.display.update()


def json_loader(filename):
    """
    LOADER
    legacy json loader
    Takes a filename and sys_path, opens a json file

    Returns a python dict
    """
    an_json = al.load_annotations_from_json_file(filename)
    if len(an_json) == 0:
        print("failed to load tracks")
        return None
    return an_json
