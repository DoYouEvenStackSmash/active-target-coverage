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

import sys
import json
import pygame
import pygame.gfxdraw
import time
import os

from drawing_functions import *

LCTRL = 64


def circular_test(screen, sensing_agent, environment):
    target_angles = [
        0,
        -0.5235987755982988,
        -0.7853981633974483,
        -1.0471975511965976,
        -1.5707963267948966,
        -2.0943951023931953,
        -2.356194490192345,
        -2.6179938779914944,
        -3.141592653589793,
        2.6179938779914944,
        2.356194490192345,
        2.0943951023931953,
        1.5707963267948966,
        1.0471975511965976,
        0.7853981633974483,
        0.5235987755982988,
        0,
    ]
    # target_angles = [
    #     0,
    #     -0.5235987755982988,
    #     -0.7853981633974483,
    #     -1.0471975511965976,
    #     -1.5607963267948966,
    #     -1.5607963267948966,
    #     -1.0471975511965976,
    #     -0.7853981633974483,
    #     -0.5235987755982988,
    #     0,
    #     0.5235987755982988,
    #     0.7853981633974483,
    #     1.0471975511965976,
    #     1.5707963267948966,
    #     1.0471975511965976,
    #     0.7853981633974483,
    #     0.5235987755982988,
    #     0,
    # ]
    target_angles.reverse()
    target_points = [mfn.pol2car((400, 400), 100, i) for i in target_angles]
    last_pt = None
    for i in range(2):
        for pt in target_points:
            # if pt == last_pt:
            #     continue
            last_pt = pt
            pafn.clear_frame(screen)
            for i in range(15):
                r, t = sensing_agent.tracker_query()
                sensing_agent.reposition(r, t)
                est = sensing_agent.obj_tracker.add_predictions()
                curr_pt, pred_pt = sensing_agent.estimate_next_detection()

                if len(pred_pt):
                    pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
                    pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                    pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])

                draw_sensing_agent(screen, sensing_agent)
                pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
                pygame.display.update()
                time.sleep(0.1)
            environment.targets[0].origin = pt
            environment.visible_targets()
            time.sleep(0.9)
        target_points.reverse()

    e = sensing_agent.export_tracks()
    f = open("out.json", "w")
    f.write(json.dumps(e, indent=2))
    f.close()
    sys.exit()


def single_agent_mouse_test(screen, sensing_agent, environment):
    pt = None
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                e = sensing_agent.export_tracks()
                f = open("out.json", "w")
                f.write(json.dumps(e, indent=2))
                f.close()
                sys.exit()
            pt = pygame.mouse.get_pos()
            if last_pt == pt:
                continue

            last_pt = pt
            print(pt)

            pafn.clear_frame(screen)

            # agent_update(sensing_agent)
            r, t = sensing_agent.tracker_query()
            sensing_agent.reposition(r, t)

            curr_pt, pred_pt = sensing_agent.estimate_next_detection()

            if len(pred_pt):
                pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
                pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])

            for k, sensing_agent in environment.agents.items():
                draw_sensing_agent(screen, sensing_agent)
            pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
            environment.targets[0].origin = pt
            environment.visible_targets()
            pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
            pygame.display.update()


def stepwise_single_agent_test(screen, sensing_agent, environment):
    pt = None
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == LCTRL:
                    pafn.clear_frame(screen)
                    draw_sensing_agent(screen, sensing_agent)
                    curr_pt, pred_pt = sensing_agent.estimate_next_detection()
                    if len(pred_pt):
                        # print((curr_pt,pred_pt))
                        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
                        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                        pafn.frame_draw_line(
                            screen, (curr_pt, pred_pt), pafn.colors["white"]
                        )
                    pygame.display.update()
                    continue

                else:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    pt = pygame.mouse.get_pos()
                    # if last_pt == pt:
                    #     continue

                    last_pt = pt
                    print(pt)

                    pafn.clear_frame(screen)
                    r, t = sensing_agent.tracker_query()
                    sensing_agent.reposition(r, t)

                    curr_pt, pred_pt = sensing_agent.estimate_next_detection()

                    if len(pred_pt):
                        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
                        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                        pafn.frame_draw_line(
                            screen, (curr_pt, pred_pt), pafn.colors["white"]
                        )

                    # for k, sensing_agent in environment.agents.items():
                    draw_sensing_agent(screen, sensing_agent)
                    pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
                    environment.targets[0].origin = pt
                    environment.visible_targets()

                    pygame.display.update()