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
from StreamingAnnotations import StreamingAnnotations as sann
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment

import sys
import json
import pygame
import pygame.gfxdraw
import time
import os

from drawing_functions import *

from support.file_loader import *
COLLISION_THRESHOLD = 10
VERBOSE = True
SAMPLE_RATE = 400
LALT = 256
LSHIFT = 1
LCTRL = 64
SPACE = 32
OFFT = 20
SPLINE_COUNT = 2


def grid_sensing_agent(screen, sensing_agent, points):
    draw_sensing_agent(screen, sensing_agent)
    pygame.display.update()
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == SPACE:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    pafn.clear_frame(screen)
                    sensing_agent.translate_agent(p)
                    draw_sensing_agent(screen, sensing_agent)
                    pygame.display.update()
                    continue
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    pafn.clear_frame(screen)
                    rotation = sensing_agent.rotate_agent(p)
                    draw_sensing_agent(screen, sensing_agent)
                    pygame.display.update()
                elif pygame.key.get_mods() == LALT:  # estimate
                    for p in points:
                        # p = pygame.mouse.get_pos()
                        visible = sensing_agent.is_visible(p)
                        if visible:
                            pafn.frame_draw_dot(screen, p, pafn.colors["green"], 8, 6)
                        else:
                            pafn.frame_draw_dot(screen, p, pafn.colors["red"], 8, 6)
                    
                elif pygame.key.get_mods() == LCTRL:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    pt = sensing_agent.transform_to_local_detection_coord(p)
                    pt2 = sensing_agent.transform_to_local_sensor_coord((0,0),pt)
                    detectable, flag = sensing_agent.is_detectable(pt2)                    
                    if detectable:
                        print(f"p: {p}\tpt3: {pt2}")
                        pafn.frame_draw_dot(screen, p, pafn.colors["cyan"])
                    else:
                        pafn.frame_draw_dot(screen, p, pafn.colors["red"])
                    
                    print(pt2)
                    continue

                for p in points:
                    pt = sensing_agent.transform_to_local_detection_coord(p)

                    pt2 = sensing_agent.transform_to_local_sensor_coord((0,0),pt)
                    pt3 = sensing_agent.transform_to_global_coord(pt)

                    detectable, flag = sensing_agent.is_detectable(pt2)
                    
                    if detectable:
                        print(f"p: {p}\tpt3: {pt2}")
                        pafn.frame_draw_dot(screen, p, pafn.colors["cyan"])
                    else:
                        pafn.frame_draw_dot(screen, p, pafn.colors["red"])
                    
                pygame.display.update()

def init_test_agent(origin, orientation, _id=0):
    sensing_agent = SensingAgent()
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
    sensor.fov_width = 2 * np.pi / 5

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
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    p = load_json_file(sys.argv[1])
    theta, r = mfn.car2pol(p[1], p[0])
    start_pt = mfn.pol2car(p[0], 50, theta)

    sensing_agent = init_test_agent((500,500),p[0], "A")
    grid_sensing_agent(screen, sensing_agent, p)
    
if __name__ == "__main__":
  main()