#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link

import collections

# from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann

# from Scene import *
import json

# import pygame
# import numpy as np
import sys

# import time

from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
from Target import Target
from Environment import Environment
import pygame
import time


def draw_coordinate_frame(screen, S):
    """
    Helper function for displaying the curved coordinate fov of agent A
    Does not return
    """
    levels = 5
    coord_frame = S.get_visible_fov(levels)
    detect_frame = S.get_detectable_bounds(levels)
    for c in range(levels):
        for i in range(1, len(coord_frame[c])):
            pafn.frame_draw_line(
                screen, (coord_frame[c][i - 1], coord_frame[c][i]), pafn.colors["white"]
            )

    for endpoint in coord_frame[-1]:
        pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors["white"])

    for endpoint in detect_frame[-1]:
        pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors["tangerine"])


def draw_all_normals(screen, R):
    """
    Render function for all coordinate frames in a chain
    Does not return
    """
    n = R.get_normals()
    pafn.frame_draw_line(
        screen, (R.get_rotation_center(), n[0]), pafn.colors["tangerine"]
    )
    pafn.frame_draw_line(screen, (R.get_rotation_center(), n[1]), pafn.colors["yellow"])
    pafn.frame_draw_dot(screen, R.get_endpoint(), pafn.colors["white"])


def draw_all_links(screen, link):
    """
    Render function for all polygon links in the chain
    Does not return
    """
    points = link.get_points()

    pafn.frame_draw_polygon(screen, points, pafn.colors["red"])


def draw_rigid_body(screen, R):
    draw_all_normals(screen, R)
    draw_all_links(screen, R)


def draw_sensing_agent(screen, SA):
    ex, s = SA.get_agent()
    draw_coordinate_frame(screen, s)
    draw_rigid_body(screen, ex)


def repeatable_sensing_agent_test(screen, E):
    step_size = 15
    destinations = []
    origin = (600, 500)
    for i in range(25):
        x, y = origin
        # destinations.append((x, y - step_size * i))
        destinations.append((x - 300, y - step_size * i))
    destinations.reverse()
    for i in reversed(destinations):
        destinations.append(i)

        # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
        # Tlist[t].origin = pt
    ptr = E.targets[0].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])

    draw_sensing_agent(screen, E.agent)
    pygame.display.update()
    time.sleep(1)

    translation_path = destinations
    print("running\n")

    rotate_counter = 0
    for pt in translation_path[1:]:
        pafn.clear_frame(screen)
        # pafn.frame_draw_dot(screen, o, pafn.colors["green"])

        E.agent.move_to_next()
        cpt, npt = E.agent.query_tracker_for_prediction()
        # print(npt)

        if len(npt):
            pafn.frame_draw_dot(screen, npt, pafn.colors["yellow"])
            pafn.frame_draw_line(screen, (cpt, npt), pafn.colors["white"])
        pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
        draw_sensing_agent(screen, E.agent)
        E.targets[0].origin = pt
        E.visible_targets()

        pygame.display.update()
        time.sleep(0.1)
    e = E.agent.export_tracks()
    f = open("out.json", "w")
    f.write(json.dumps(e, indent=2))
    f.close()
    # time.sleep(1)


def main():
    pygame.init()

    screen = pafn.create_display(1000, 1000)
    origin = (500, 500)
    ox, oy = origin
    scale = 3

    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
    mpt = gfn.get_midpoint(opts[0], opts[1])
    mpt2 = gfn.get_midpoint(mpt, opts[2])
    ap = Polygon(opts)
    rb = RigidBody(ref_origin=mpt, ref_center=mpt2, endpoint=opts[2], rigid_body=ap)
    rb.prev = rb
    rb.rotate_body(origin=rb.ref_center, theta=-3 * np.pi / 4)
    # rb.rel_theta = 3 * np.pi / 2
    s = Sensor()
    s.fov_width = 3 * np.pi / 5
    s.origin = rb.ref_endpoint
    s.fov_theta = rb.rel_theta
    A = SensingAgent(rb, s)
    A.obj_tracker = ObjectTrackManager()
    T = Target((500, 550))
    E = Environment(A, [T])

    repeatable_sensing_agent_test(screen, E)


if __name__ == "__main__":
    main()
