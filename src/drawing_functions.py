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


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


DRAW_HORIZONTAL = False
OUTLINE = False


def draw_coordinate_frame(screen, sensor):
    """
    Helper function for displaying the sensor field of view
    Does not return
    """
    levels = 5
    coord_frame = sensor.get_visible_fov(levels)
    detect_frame = sensor.get_detectable_bounds(levels)
    if OUTLINE:
        for i in range(1, len(coord_frame[-1])):
            pafn.frame_draw_line(
                screen,
                (coord_frame[-1][i - 1], coord_frame[-1][i]),
                pafn.colors["black"],
            )
        # for endpoint in coord_frame[-1][:]:
        #   pafn.frame_draw_line(screen, (sensor.get_origin(), endpoint), pafn.colors['white'])
    else:
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
    pafn.frame_draw_dot(screen, rigid_body.get_center(), pafn.colors["white"])


def draw_all_links(screen, link, color=None):
    """
    Render function for all polygon links in the chain
    Does not return
    """
    if color == None:
        color = pafn.colors["indigo"]
    points = link.get_points()
    pafn.frame_draw_filled_polygon(screen, points, color)
    pafn.frame_draw_polygon(screen, points, pafn.colors["black"])
    # pafn.frame_draw_dot(screen, link.get_center(), pafn.colors['white'])


def draw_rigid_body(screen, rigid_body):
    """
    Wrapper for rendering all components of a rigid body
    """
    draw_all_normals(screen, rigid_body)
    draw_all_links(screen, rigid_body, rigid_body.color)


def draw_sensing_agent(screen, sensing_agent):
    """
    Wrapper for rendering all components of an agent
    """
    exoskeleton, sensor = sensing_agent.get_components()
    draw_coordinate_frame(screen, sensor)
    draw_rigid_body(screen, exoskeleton)
