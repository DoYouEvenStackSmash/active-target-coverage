#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link
from support.CollisionDetection import CollisionDetection

import collections

# from aux_functions import *
# from Dataloader import Dataloader
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


def adjust_angle(theta):
    """
    adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]
    """
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


DRAW_BOUNDARIES = True  # draw the threshold regions for visibility
DRAW_HORIZONTAL = False
OUTLINE = True  # draw only the outlined fov of the agent
GRID = False  # draw the cartesian plane around the agent

LINE = True  # allow a straight line to be drawn between last detection and next prediction
DELAY = 3  # delay to draw marked points

INVERTED = 1
MARKERS = True  # draw markers for the predictions

frame_colors = [pafn.colors["white"], pafn.colors["black"]]
color_grade = [
    pafn.colors["white"],
    pafn.colors["tangerine"],
    pafn.colors["lightslategray"],
    pafn.colors["dimgray"],
    pafn.colors["black"],
    pafn.colors["black"],
    pafn.colors["black"],
]


def draw_coordinate_frame(screen, sensor, levels=5):
    """
    Helper function for displaying the sensor field of view
    Does not return
    Args:
        screen: A pygame screen object for drawing
        sensor (Sensor): the sensor object to be drawn
        levels (int): the number of rows to be drawn
    """
    coord_frame = sensor.get_visible_fov(levels)
    detect_frame = sensor.get_detectable_bounds(levels)

    if OUTLINE:  # draw skeletal outline only
        for i in range(1, len(coord_frame[-1])):
            pafn.frame_draw_line(
                screen,
                (coord_frame[-1][i - 1], coord_frame[-1][i]),
                frame_colors[INVERTED],
            )
    else:  # draw full sensor field of view
        for c in range(levels):
            for i in range(1, len(coord_frame[c])):
                pafn.frame_draw_line(
                    screen,
                    (coord_frame[c][i - 1], coord_frame[c][i]),
                    frame_colors[INVERTED],
                )
        for endpoint in coord_frame[-1]:
            pafn.frame_draw_line(
                screen, (sensor.get_origin(), endpoint), pafn.colors["dimgray"]
            )

    # draw threshold regions
    if DRAW_BOUNDARIES:
        for endpoint in detect_frame[-1][:2]:
            pafn.frame_draw_line(
                screen, (sensor.get_origin(), endpoint), pafn.colors["tangerine"]
            )
        for endpoint in detect_frame[-1][2:]:
            pafn.frame_draw_line(
                screen, (sensor.get_origin(), endpoint), pafn.colors["cyan"]
            )


def draw_all_normals(screen, rigid_body):
    """
    Render function for all coordinate frames in a chain
    Does not return
    Args:
        screen: A pygame screen object for drawing
        rigid_body (RigidBody): A rigid body class object
    """
    n = rigid_body.get_normals()
    pafn.frame_draw_line(
        screen, (rigid_body.get_center(), n[0]), pafn.colors["tangerine"]
    )
    pafn.frame_draw_line(screen, (rigid_body.get_center(), n[1]), pafn.colors["yellow"])
    pafn.frame_draw_dot(screen, rigid_body.get_center(), pafn.colors["white"])


def draw_all_links(screen, link, color=None):
    """
    Render function for all polygon links in the chain
    Does not return
    Args:
        screen: A pygame screen object for drawing
        link (Link): An abstraction which contains a polygon
        color: A custom color for filling in the link polygon
    """
    if color == None:
        color = pafn.colors["indigo"]
    points = link.get_points()
    pafn.frame_draw_filled_polygon(screen, points, color)
    pafn.frame_draw_polygon(screen, points, frame_colors[INVERTED])
    pafn.frame_draw_dot(screen, link.get_center(), pafn.colors["white"])


def draw_rigid_body(screen, rigid_body):
    """
    Wrapper function for rendering all components of a rigid body
    Args:
        screen: a pygame screen object for drawing
        rigid_body (RigidBody): Abstraction containing attributes of angle and points
    """
    draw_all_normals(screen, rigid_body)
    draw_all_links(screen, rigid_body, rigid_body.color)
    if GRID:
        draw_body_grid(screen, rigid_body)
    # pafn.frame_draw_bold_line(screen, rigid_body.get_horizontal_axis(), pafn.colors["black"])
    # pafn.frame_draw_bold_line(screen, rigid_body.get_vertical_axis(), pafn.colors["black"])


def draw_body_grid(screen, rigid_body):
    axes = rigid_body.get_grid()
    for ax in axes:
        pafn.frame_draw_line(screen, ax, frame_colors[INVERTED])


def draw_sensing_agent(screen, sensing_agent):
    """
    Wrapper for rendering all components of an agent
    Args:
        screen: a pygame screen object for drawing
        sensing_agent (SensingAgent): the sensing agent to be drawn
    """
    exoskeleton, sensor = sensing_agent.get_components()
    draw_coordinate_frame(screen, sensor)
    # draw_body_coordinate_frame(screen, exoskeleton)
    draw_rigid_body(screen, exoskeleton)


def render_predictions(screen, sensing_agent):
    """
    renders an agents predictions if applicable
    """
    curr_pt, pred_pt = (), ()
    arr = sensing_agent.estimate_next_detection()
    if not len(arr):
        return
    for i in range(len(arr)):
        curr_pt = arr[i][0]
        pred_pt = arr[i][1]
        if len(pred_pt):
            # pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"], 5,10)
            pafn.frame_draw_cross(screen, pred_pt, pafn.colors["yellow"], 20)
            if LINE:
                pafn.frame_draw_bold_line(screen, (curr_pt, pred_pt), pafn.colors["white"])


def render_path(screen, path, color):
    """
    renders a sequence of points on the screen
    """
    for i in range(1, len(path)):
        pafn.frame_draw_bold_line(screen, (path[i - 1], path[i]), color)


def import_agent_record(screen, agent_record):
    """
    Imports a LOCO formatted json

    draws the paths on the screen
    """

    fov_width = agent_record["sensor_params"]["fov_width"]
    fov_radius = agent_record["sensor_params"]["fov_radius"]

    trackmap = agent_record["trackmap"]
    lt = agent_record["linked_tracks"]

    get_center = lambda state: (state["position"]["x"], state["position"]["y"])
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
            state = states[anno["state_id"] - 1]
            x, y, w, h = anno["bbox"]
            rect_x, rect_y = anno["position"]["x"], anno["position"]["y"]
            theta, rad = mfn.car2pol((0, 0), (rect_x, rect_y))

            # theta = (x - 50) / Sensor.WINDOW_WIDTH * fov_width
            theta = adjust_angle(get_orientation(state) + theta)
            # r = y
            pt = mfn.pol2car(get_center(state), rad, theta)
            pts.append(pt)

        render_path(screen, pts, color)
        pygame.display.update()


def accumulate_predictions(sensing_agent, curr_pts, pred_pts):
    """
    Accumulates the last detection and predicted next detection points
    for rendering.
    """
    curr_pt, pred_pt = (), ()
    arr = sensing_agent.estimate_next_detection()
    if len(arr):
        for i in range(len(arr)):
            curr_pt = arr[i][0]
            pred_pt = arr[i][1]

            if len(pred_pt):
                curr_pts.append(curr_pt)
                pred_pts.append(pred_pt)
            else:
                curr_pts.append(curr_pt)
                pred_pts.append(curr_pt)
    else:
        return


def environment_agent_update(environment, FORCE_UPDATE=False):
    """
    Allows agents to make their predictions and move if necessary
    """
    for k in environment.agents:
        sensing_agent = environment.agents[k]
        if sensing_agent.IS_DEAD:
            continue
        if sensing_agent.ALLOW_PREDICTION == FORCE_UPDATE:
            r, t = sensing_agent.tracker_query()
            sensing_agent.reposition(r, t)
            sensing_agent.heartbeat()


def environment_agent_illustration(
    screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts
):
    """
    Draws nice things (Agents, marked points, predictions, etc) on the screen
    """
    for k in environment.agents:
        sensing_agent = environment.agents[k]
        render_predictions(screen, sensing_agent)
        # demo rendering
        accumulate_predictions(sensing_agent, curr_pts, pred_pts)
        for i in range(max(0, len(marked_pts) - 5), len(marked_pts)):
            pafn.frame_draw_dot(
                screen,
                marked_pts[i],
                color_grade[len(marked_pts) - i],
                0,
                (1 - (len(marked_pts) - i) / 5) * 10,
            )
        # if len(curr_pts):
        #   pafn.frame_draw_cross(screen, curr_pts[-1], pafn.colors["tangerine"], 20)
        if MARKERS:
            for idx in range(
                max(0, len(pred_pts) - int(measurement_rate * DELAY)), len(pred_pts), 3
            ):
                pafn.frame_draw_dot(
                    screen, pred_pts[idx], sensing_agent.exoskeleton.color, 0, 4
                )

        draw_sensing_agent(screen, sensing_agent)

def environment_target_illustration(screen, environment):
    for t in environment.targets:
        attr = t.attributes
        cx,cy = attr[0],attr[1]
        w,h = attr[2], attr[3]
        box = [(cx-w/2,cy-h/2), (cx + w/2,cy - h/2), (cx + w/2,cy + h/2), (cx - w/2,cy + h/2)]
        pafn.frame_draw_polygon(screen, box)