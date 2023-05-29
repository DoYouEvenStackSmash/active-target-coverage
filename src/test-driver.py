#!/usr/bin/python3

import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
import pygame

import collections

from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment
from single_agent_tests import *
import sys
import json
import pygame
import pygame.gfxdraw
import time
import os

from drawing_functions import *

LCTRL = 64


def init_sensing_agent(
    sensing_agent=SensingAgent(), origin=(0, 0), _id=0, orientation=(500, 400)
):
    ox, oy = origin
    scale = 2
    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
    # print(opts)

    mpt = gfn.get_midpoint(opts[0], opts[1])
    mpt2 = gfn.get_midpoint(mpt, opts[2])
    ap = Polygon(opts)
    rb = RigidBody(
        parent_agent=sensing_agent,
        ref_origin=mpt,
        ref_center=mpt2,
        endpoint=opts[2],
        rigid_link=ap,
    )
    sensor = Sensor(parent_agent=sensing_agent)
    sensor.fov_width = 3 * np.pi / 5

    sensing_agent.exoskeleton = rb
    sensing_agent.exoskeleton.states = []

    sensing_agent.centered_sensor = sensor
    sensing_agent.obj_tracker = ObjectTrackManager()
    sensing_agent.obj_tracker.linked_tracks = []
    sensing_agent.obj_tracker.layers = []
    sensing_agent.obj_tracker.trackmap = []
    sensing_agent.obj_tracker.global_track_store = {}

    sensing_agent.obj_tracker.parent_agent = sensing_agent
    sensing_agent._id = _id
    rotation = sensing_agent.rotate_agent(orientation)
    return sensing_agent


def main():
    origins = [(900, 600), (400, 700), (400, 400)]

    environment = Environment()
    sensing_agent = init_sensing_agent(SensingAgent(), origins[2], "A")
    # sensing_agent.ALLOW_TRANSLATION = False

    sensing_agent_2 = init_sensing_agent(SensingAgent(), origins[1], "B")

    target = Target((950, 950), _id=41)
    # target_2 = Target(sensing_agent_2.exoskeleton.origin, _id=42)
    environment.add_target(target)
    # environment.add_target(target_2)
    environment.agents["A"] = sensing_agent
    # environment.agents["B"] = sensing_agent_2
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    # sensing_agent.ALLOW_ROTATION = False
    sensing_agent.ALLOW_TRANSLATION = False
    circular_test(screen, sensing_agent, environment)
    # stepwise_single_agent_test(screen, sensing_agent, environment)
    # single_agent_mouse_test(screen, sensing_agent, environment)
    # multi_agent_mouse_test(screen, environment)


if __name__ == "__main__":
    main()
