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

from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann

import json

import sys

from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
import pygame
import time


class Environment:
    """
    Mock environment for simulating the world
    """

    def __init__(
        self,
        agent=None,  # single agent for backwards compatibility
        agents={},  # dictionary of agents, accessible by their unique identifiers
        targets=[],  # list of targets in the world
        counter=0,  # global counter for synchronizing "time"
    ):
        self._agent = agent
        self.agents = agents
        self.targets = targets
        self.counter = counter

    def visible_targets(self):
        """
        Determines visibility between agents and targets
        Does not return
        """
        pairs = []
        sortkey = lambda x: x[2]
        frame_id = "frame_" + str(self.counter)
        self.counter += 1
        updates = {}

        for k in self.agents:
            self.agents[k].heartbeat()
            updates[k] = []
            for target in self.targets:
                d = mfn.euclidean_dist(self.agents[k].get_origin(), target.get_origin())
                pairs.append((self.agents[k]._id, target, d))

        pairs = sorted(pairs, key=sortkey)
        # TODO: short circuit the for loop, minimum number of updates?

        # iterate through all pairs and determine which agents can see which targets
        for c in range(len(pairs)):
            if pairs[c][2] > self.agents[pairs[c][0]].get_fov_radius():
                continue
            if self.agents[pairs[c][0]].is_visible(pairs[c][1].get_origin()):
                updates[pairs[c][0]].append(pairs[c][1])

        # update the trackers of all agents
        for k in updates:
            self.agents[k].new_detection_layer(frame_id, updates[k])

    def add_target(self, T):
        """
        Add a target to the world
        """
        self.targets.append(T)

    def transform_from_local_coord(self, x, y, w=1, h=1):
        """
        Transforms a bbox from Sensor local coordinates to world coordinates
        returns a Point
        """
        org_theta = mfn.correct_angle(self.agent.fov_theta)

        rh = org_theta - self.agent.sensor.fov_width / 2
        lh = org_theta + self.agent.sensor.fov_width / 2
        theta = (x / Sensor.WINDOW_WIDTH) * self.agent.sensor.fov_width
        # print(f"is_not_visible: {lh}:{theta}:{rh}")
        ratio = theta - 0.5
        theta = lh - theta
        theta = mfn.correct_angle(theta)

        r = y
        return mfn.pol2car(self.agent.get_origin(), r, theta)
