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
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment
import sys
import json
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


def import_loco_fmt(s, sys_path):
    """
    Imports a LOCO formatted json

    Returns a json of some sort
    """

    # set up trackmap for accessing tracks
    # self.imported = True
    # trackmap = s['trackmap']
    # lt = s['linked_tracks']
    # for i,track_id in enumerate(trackmap):
    #   if track_id not in self.global_track_store or track_id == -1:
    #     self.global_track_store[track_id] = ObjectTrack(track_id, lt[i]['category_id'])
    #     self.global_track_store[track_id].class_id = lt[i]['category_id']
    #   else: #already present
    #     continue

    # load image filenames
    # images = s['images']
    # # construct file dict for accessing file ids
    # # construct sys_paths list for convenience
    # # initialize layers to populate with YoloBoxes
    # for i,imf in enumerate(images):
    #   self.filenames.append(imf['file_name'])
    #   self.sys_paths.append(sys_path)
    #   self.fdict[imf['file_name']] = i
    #   self.layers.append([])
    #   self.img_centers.append(tuple((int(imf['width']/2), int(imf['height']/2))))

    # load annotations
    steps = s["annotations"]
    # for st in steps:
    #   # skip step if track is invalid
    #   if trackmap[st['trackmap_index']] == -1:
    #     continue
    #   track = self.get_track(trackmap[st['trackmap_index']])
    #   yb = YoloBox( track.class_id,
    #                 st['bbox'],
    #                 f'{self.filenames[st["image_id"]][:-3]}txt',
    #                 self.img_centers[st["image_id"]])

    #   # add YoloBox to the appropriate layer based on the image filename
    #   self.layers[self.fdict[self.filenames[st['image_id']]]].append(yb)
    #   # add the yolobox to the correct track
    #   track.add_new_step(yb,0)
    # bboxes = []

    return steps


def drive(screen, origin, pt):
    rad, r = mfn.car2pol(origin, pt)
    if rad < np.pi / 2 and rad > -np.pi / 2:
        print("turn right")
        pafn.frame_draw_bold_line(screen, (origin, pt), pafn.colors["green"])
    else:
        print("turn left")
        pafn.frame_draw_bold_line(screen, (origin, pt), pafn.colors["red"])


def normalize(rel_x, rel_y, abs_x_max=1000):
    # print(rel_x)
    rel_x = rel_x / 100 * abs_x_max
    rel_y = 500
    return (rel_x, rel_y)


def draw_outline(screen):
    crosshairs = [[(485, 500), (515, 500)], [(500, 485), (500, 515)]]
    box = [(100, 100), (100, 900), (900, 900), (900, 100)]
    for i in crosshairs:
        pafn.frame_draw_line(screen, i, pafn.colors["magenta"])
    pafn.frame_draw_polygon(screen, box, pafn.colors["indigo"])


def draw_coordinate_frame(screen, sensor):
    """
    Helper function for displaying the sensor field of view
    Does not return
    """
    levels = 5
    coord_frame = sensor.get_visible_fov(levels)
    detect_frame = sensor.get_detectable_bounds(levels)
    for c in range(levels):
        for i in range(1, len(coord_frame[c])):
            pafn.frame_draw_line(
                screen, (coord_frame[c][i - 1], coord_frame[c][i]), pafn.colors["white"]
            )

    for endpoint in coord_frame[-1]:
        pafn.frame_draw_line(
            screen, (sensor.get_origin(), endpoint), pafn.colors["white"]
        )

    for endpoint in detect_frame[-1]:
        pafn.frame_draw_line(
            screen, (sensor.get_origin(), endpoint), pafn.colors["tangerine"]
        )


def draw_all_normals(screen, rigid_body):
    """
    Render function for all coordinate frames in a chain
    Does not return
    """
    n = rigid_body.get_normals()
    pafn.frame_draw_line(
        screen, (rigid_body.get_center(), n[0]), pafn.colors["tangerine"]
    )
    pafn.frame_draw_line(screen, (rigid_body.get_center(), n[1]), pafn.colors["yellow"])
    pafn.frame_draw_dot(screen, rigid_body.get_endpoint(), pafn.colors["white"])


def draw_all_links(screen, link):
    """
    Render function for all polygon links in the chain
    Does not return
    """
    points = link.get_points()
    pafn.frame_draw_polygon(screen, points, pafn.colors["red"])


def draw_rigid_body(screen, rigid_body):
    """
    Wrapper for rendering all components of a rigid body
    """
    draw_all_normals(screen, rigid_body)
    draw_all_links(screen, rigid_body)


def draw_sensing_agent(screen, sensing_agent):
    """
    Wrapper for rendering all components of an agent
    """
    exoskeleton, sensor = sensing_agent.get_components()
    draw_coordinate_frame(screen, sensor)
    draw_rigid_body(screen, exoskeleton)


def do_rel_detection(screen, sensing_agent):
    # pafn.clear_frame(screen)
    curr_pt, pred_pt = sensing_agent.estimate_rel_next_detection()
    print((curr_pt, pred_pt))
    if not len(pred_pt):
        return
    if len(pred_pt):
        prev_pt = [pred_pt[0], pred_pt[1]]
        curr_pt = normalize(curr_pt[0], curr_pt[1])
        pred_pt = normalize(pred_pt[0], pred_pt[1])
        status, flags = sensing_agent.is_rel_detectable(prev_pt)
        if flags == Sensor.ANGULAR:
            drive(screen, (500, 500), pred_pt)

        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
        pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])


turn_count = 0


def do_est_rotation(screen, sensing_agent):
    est_rotation = ()
    est_rotation, est_translation = sensing_agent.estimate_next_rotation()

    if est_rotation == None:
        print("not_rotating")
        return
    global turn_count
    if est_rotation != None:
        turn_count += 1

        print(est_rotation)
        rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
        sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
        sensing_agent.exoskeleton.rel_theta += rotation


def yolo_test(screen, sensing_agent):
    s = json_loader("out.json")
    annos = import_loco_fmt(s, None)

    pts = [(450, 500), (350, 500), (250, 500), (350, 500), (450, 500)]
    origin = (500, 500)
    abs_x_max = 1000
    abs_x_min = 0
    x_max = 100
    x_min = 0
    r = 50
    counter = 0
    crosshairs = [[(485, 500), (515, 500)], [(500, 485), (500, 515)]]
    box = [(100, 100), (100, 900), (900, 900), (900, 100)]
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == SPACE:
                    continue
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    pafn.clear_frame(screen)
                    curr_pt, pred_pt = sensing_agent.estimate_rel_next_detection()
                    print((curr_pt, pred_pt))
                    if len(pred_pt):
                        curr_pt = normalize(curr_pt[0], curr_pt[1])
                        pred_pt = normalize(pred_pt[0], pred_pt[1])
                        drive(screen, (500, 500), pred_pt)
                        # print((curr_pt,pred_pt))
                        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
                        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
                        pafn.frame_draw_line(
                            screen, (curr_pt, pred_pt), pafn.colors["white"]
                        )
                    draw_sensing_agent(screen, sensing_agent)
                    draw_outline(screen)
                    pygame.display.update()
                    continue
                elif pygame.key.get_mods() == LALT:  # estimate
                    est_rotation = ()
                    est_rotation = sensing_agent.estimate_next_rotation()

                    # pred_rotation = sensing_agent.exoskeleton.get_relative_rotation(pt)
                    if len(est_rotation):
                        pafn.clear_frame(screen)
                        est_rotation = est_rotation[0]
                        print(est_rotation)
                        rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
                        sensing_agent.obj_tracker.add_angular_displacement(
                            0, -est_rotation
                        )
                        sensing_agent.exoskeleton.rel_theta += rotation

                    draw_sensing_agent(screen, sensing_agent)
                    draw_outline(screen)
                    pygame.display.update()
                    # sys.exit()
                    continue
                elif pygame.key.get_mods() == LCTRL:
                    for a in annos:
                        x, y = a["bbox"][0], 50
                        bbox = [x, y, 1, 1]
                        yb = sann.register_annotation(0, bbox, f"frame_{counter}")
                        sensing_agent.obj_tracker.add_new_layer([yb])
                        sensing_agent.obj_tracker.process_layer(
                            len(sensing_agent.obj_tracker.layers) - 1
                        )
                        pafn.clear_frame(screen)

                        do_rel_detection(screen, sensing_agent)
                        draw_sensing_agent(screen, sensing_agent)
                        draw_outline(screen)
                        do_est_rotation(screen, sensing_agent)

                        pygame.display.update()

                        time.sleep(0.2)
                    print(turn_count)
                    sys.exit()
                    continue
                    # pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
                else:
                    pafn.clear_frame(screen)
                    draw_outline(screen)
                    draw_sensing_agent(screen, sensing_agent)
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    x = (p[0] / abs_x_max) * x_max
                    y = 50
                    pt = (x, y)
                    bbox = [x, y, 1, 1]
                    yb = sann.register_annotation(0, bbox, f"frame_{counter}")
                    sensing_agent.obj_tracker.add_new_layer([yb])
                    sensing_agent.obj_tracker.process_layer(
                        len(sensing_agent.obj_tracker.layers) - 1
                    )
                    pygame.display.update()
                    print(pt)
                    continue


def sa_setup():
    sensing_agent = SensingAgent()
    ox, oy = 300, 300
    scale = 2
    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
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

    sensing_agent.exoskeleton = rb
    sensing_agent.sensor = sensor
    sensing_agent.obj_tracker = ObjectTrackManager()

    sensing_agent.obj_tracker.parent_agent = sensing_agent
    return sensing_agent


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pygame.display.update()
    sa = sa_setup()
    yolo_test(screen, sa)


if __name__ == "__main__":
    main()
