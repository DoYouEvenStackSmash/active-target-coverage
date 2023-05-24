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


def agent_update(sensing_agent):
    """
    Updates the pose of a single agent
    """
    est_rotation, est_translation = sensing_agent.estimate_pose_update()

    if est_rotation != None:
        if est_rotation > np.pi or est_rotation < -np.pi:
            print("rotation OOB")
        rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
        direction = 1 if est_rotation > 0 else -1
        print(f"sensing_agent theta: {sensing_agent.exoskeleton.rel_theta}")
        sensing_agent.exoskeleton.rel_theta += rotation
        if sensing_agent.exoskeleton.rel_theta < -np.pi:
            sensing_agent.exoskeleton.rel_theta = (
                2 * np.pi + sensing_agent.exoskeleton.rel_theta
            )
        if sensing_agent.exoskeleton.rel_theta > np.pi:
            sensing_agent.exoskeleton.rel_theta = (
                -2 * np.pi + sensing_agent.exoskeleton.rel_theta
            )
        sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation, direction)
    if est_translation != None:
        translation = sensing_agent.apply_translation_to_agent(est_translation)
        sensing_agent.obj_tracker.add_linear_displacement(-translation, 0)


def multi_agent_mouse_test(screen, environment):
    pt = None
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                for _id in environment.agents:
                    sensing_agent = environment.agents[_id]
                    e = sensing_agent.export_tracks()
                    f = open(f"{_id}_out.json", "w")
                    f.write(json.dumps(e, indent=2))
                    f.close()
                sys.exit()
            pt = pygame.mouse.get_pos()
            if last_pt == pt:
                continue

            last_pt = pt
            pafn.clear_frame(screen)

            for k in environment.agents:
                agent_update(environment.agents[k])

                curr_pt, pred_pt = environment.agents[k].estimate_next_detection()

                if len(pred_pt):
                    pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
                    pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                    pafn.frame_draw_line(
                        screen, (curr_pt, pred_pt), pafn.colors["white"]
                    )

            for k in environment.agents:
                sensing_agent = environment.agents[k]
                draw_sensing_agent(screen, sensing_agent)
            environment.targets[0].origin = pt
            pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
            environment.visible_targets()
            pygame.display.update()


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

            agent_update(sensing_agent)

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


def interactive_single_agent_test(screen, sensing_agent, environment):
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

                    agent_update(sensing_agent)

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
    target_angles.reverse()
    target_points = [mfn.pol2car((400, 400), 100, i) for i in target_angles]
    last_pt = None
    for pt in target_points:
        if pt == last_pt:
            continue
        last_pt = pt
        pafn.clear_frame(screen)

        agent_update(sensing_agent)

        curr_pt, pred_pt = sensing_agent.estimate_next_detection()

        if len(pred_pt):
            pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
            pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
            pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])

        # for k, sensing_agent in environment.agents.items():
        draw_sensing_agent(screen, sensing_agent)
        pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
        pygame.display.update()
        environment.targets[0].origin = pt
        environment.visible_targets()
        time.sleep(1)
    e = sensing_agent.export_tracks()
    f = open("out.json", "w")
    f.write(json.dumps(e, indent=2))
    f.close()
    sys.exit()


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
    # circular_test(screen, sensing_agent, environment)
    # interactive_single_agent_test(screen, sensing_agent, environment)
    single_agent_mouse_test(screen, sensing_agent, environment)
    # multi_agent_mouse_test(screen, environment)


if __name__ == "__main__":
    main()
