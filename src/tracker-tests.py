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

from support.file_loader import *


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

    print(len(lt))
    for track in lt:
        pts = []
        color = None
        for step_id in track["steps"]:
            anno = annotations[step_id]
            color = anno["track_color"]
            state = states[anno["state_id"] - 1]
            x, y, w, h = anno["bbox"]
            theta = (x - 50) / Sensor.WINDOW_WIDTH * fov_width
            theta = adjust_angle(get_orientation(state) + theta)
            r = y
            pt = mfn.pol2car(get_center(state), r, theta)
            pts.append(pt)

        # render_path(screen, pts, color)
        pygame.display.update()


def constant_angular_test(screen, path, environment):
    pred_pts = []
    est_pts = []
    for i in range(0, len(path)):
        p = path[i]
        pafn.clear_frame(screen)

        for j in path[:i]:
          pafn.frame_draw_dot(screen, j, pafn.colors["magenta"])
        # pygame.display.update()
        for k in environment.agents:
            sensing_agent = environment.agents[k]
            # est = sensing_agent.add_predictions()
            
            

            
            for L in range(2):
                if i % 2:
                    break
                r, tr = sensing_agent.tracker_query()
                sensing_agent.reposition(r, tr)
                continue
                
                
                print(est)
                # print(est)
                for yb in est:
                    # print(yb.bbox)
                    x, y, w, h = yb.bbox
                    pred_pt = (x, y)

                    curr_pt = pred_pt

                    pred_pt = sensing_agent.transform_from_local_coord(x, y)

                    est_pts.append(pred_pt)

                    pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
                    pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                    pafn.frame_draw_line(
                        screen, (curr_pt, pred_pt), pafn.colors["white"]
                    )

            # pygame.display.update()
                  # time.sleep(0.06)
            # est = sensing_agent.obj_tracker.add_predictions()
            draw_sensing_agent(screen, sensing_agent)
        for ep in est_pts:
            pafn.frame_draw_dot(screen, ep, pafn.colors["tangerine"])
        for t in environment.targets:
            t.origin = p
            pafn.frame_draw_dot(screen, t.origin, pafn.colors["green"])
        pygame.display.update()
        if i < 5 or not i % 4:
          environment.visible_targets()
        time.sleep(0.01)

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                # path = {"points": [{"x":p[0], "y":p[1]} for p in  paths[0]]}
                for _id in environment.agents:
                    sensing_agent = environment.agents[_id]
                    e = sensing_agent.export_tracks()
                    import_agent_record(screen, e)
                    f = open(f"{_id}_out.json", "w")
                    f.write(json.dumps(e, indent=2))
                    f.close()
                    sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    p = load_json_file(sys.argv[1])
    theta, r = mfn.car2pol(p[1], p[0])
    start_pt = mfn.pol2car(p[0], 50, theta)

    sensing_agent = init_test_agent(start_pt, p[0], "A")
    sensing_agent.centered_sensor.fov_radius = 400
    environment = Environment()
    target = Target(p[0])
    environment.add_target(target)
    environment.agents["A"] = sensing_agent
    draw_sensing_agent(screen, sensing_agent)
    pygame.display.update()
    constant_angular_test(screen, p, environment)
    # time.sleep(4)

    print(p)


if __name__ == "__main__":
    main()
