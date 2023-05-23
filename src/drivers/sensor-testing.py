#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link

from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
from Target import Target
from Environment import Environment
import pygame
import time

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
        pafn.frame_draw_line(screen, (S.get_origin(), endpoint), pafn.colors["white"])

    for endpoint in detect_frame[-1]:
        pafn.frame_draw_line(
            screen, (S.get_origin(), endpoint), pafn.colors["tangerine"]
        )


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
    draw_coordinate_frame(screen, SA)
    draw_rigid_body(screen, ex)


def repeatable_sensor_test(screen, E):
    directions = [-np.pi, -np.pi / 2, 0, np.pi / 2]
    target_points = [(450, 450), (550, 450), (550, 550), (450, 550)]
    draw_sensing_agent(screen, E.agent)
    pygame.display.update()

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
    translation_path = destinations

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == LCTRL:
                    continue
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    for pt in translation_path[1:]:
                        pafn.clear_frame(screen)
                        # pafn.frame_draw_dot(screen, o, pafn.colors["green"])

                        # print(E.agent.is_visible(pt))
                        E.agent.estimate_movement()

                        # E.agent.rotate_to_target(pt)
                        cpt, npt = E.agent.query_tracker_for_prediction()

                        # print(npt)
                        if len(npt):
                            cpt = E.transform_from_local_coord(cpt[0], cpt[1])
                            npt = E.transform_from_local_coord(npt[0], npt[1])
                            E.agent.rotate_to_target(pt)
                            x, y, w, h = E.transform_to_local_coord(npt)
                            print((x, y))
                            pafn.frame_draw_dot(screen, npt, pafn.colors["yellow"])
                            pafn.frame_draw_line(
                                screen, (cpt, npt), pafn.colors["white"]
                            )
                        pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
                        draw_sensing_agent(screen, E.agent)
                        E.targets[0].origin = pt
                        E.visible_targets()

                        pygame.display.update()
                        time.sleep(0.5)
                    continue
                elif pygame.key.get_mods() == LALT:  # estimate
                    continue
                else:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    print(E.agent.query_sensor_visible(p))
                    E.agent.rotate_to_target(p)
                    draw_sensing_agent(screen, E.agent)
                    pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    ox, oy = 400, 400
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
    rb.fov_theta = 0
    s = Sensor()
    s.fov_width = 3 * np.pi / 5
    s.origin = rb.ref_endpoint
    A = SensingAgent(rb, s)
    T = Target((500, 550))
    E = Environment(A, [T])
    repeatable_sensor_test(screen, E)


main()
