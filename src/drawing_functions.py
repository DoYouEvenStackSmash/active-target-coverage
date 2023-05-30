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


DRAW_HORIZONTAL = False
OUTLINE = False


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
                pafn.colors["black"],
            )
    else:  # draw full sensor field of view
        for c in range(levels):
            for i in range(1, len(coord_frame[c])):
                pafn.frame_draw_line(
                    screen,
                    (coord_frame[c][i - 1], coord_frame[c][i]),
                    pafn.colors["black"],
                )
        for endpoint in coord_frame[-1]:
            pafn.frame_draw_line(
                screen, (sensor.get_origin(), endpoint), pafn.colors["dimgray"]
            )

    # draw threshold regions
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
    pafn.frame_draw_polygon(screen, points, pafn.colors["black"])
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
    draw_body_grid(screen, rigid_body)
    # pafn.frame_draw_bold_line(screen, rigid_body.get_horizontal_axis(), pafn.colors["black"])
    # pafn.frame_draw_bold_line(screen, rigid_body.get_vertical_axis(), pafn.colors["black"])


def draw_body_grid(screen, rigid_body):
    axes = rigid_body.get_grid()
    for ax in axes:
        pafn.frame_draw_line(screen, ax, pafn.colors["black"])


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
    curr_pt, pred_pt = (),()
    arr = sensing_agent.estimate_next_detection()
    if len(arr):
        curr_pt = arr[0][0]
        pred_pt = arr[0][1]
    if len(pred_pt):
        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
        pafn.frame_draw_line(
            screen, (curr_pt, pred_pt), pafn.colors["white"]
        )

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
            state = states[anno["state_id"] - 1]
            x, y, w, h = anno["bbox"]
            theta = (x - 50) / Sensor.WINDOW_WIDTH * fov_width
            theta = adjust_angle(get_orientation(state) + theta)
            r = y
            pt = mfn.pol2car(get_center(state), r, theta)
            pts.append(pt)

        render_path(screen, pts, color)
        pygame.display.update()