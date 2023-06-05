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
from StreamingAnnotations import StreamingAnnotations as sann

from Detection import *
import json

import sys

from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
import pygame
import time


class SimulationEnvironment:
    """A class for simulating the environment in which the agents operate
    Attributes:
        Agent (Agent): legacy Agent attribute
        Agents (List[Agent]): list of all agents present in the environment
        Targets (List[Target]): list of all targets present in the environment

    Mock environment for simulating the world
    """

    def __init__(
        self,
        world_origin=Position(0, 0, 0),
        agent=None,  # single agent for backwards compatibility
        agents=None,  # dictionary of agents, accessible by their unique identifiers
        targets=None,  # list of targets in the world
        counter=0,  # global counter for synchronizing "time"
    ):
        self.world_origin = world_origin
        self._agent = agent
        self.agents = agents if agents != None else {}
        self.targets = targets if targets != None else []
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
            # self.agents[k].heartbeat()
            updates[k] = []
            for target in self.targets:
                d = mfn.frobenius_dist(
                    self.agents[k].get_origin().get_cartesian_coordinates(),
                    target.get_position().get_cartesian_coordinates(),
                )
                pairs.append((self.agents[k]._id, target, d))

        pairs = sorted(pairs, key=sortkey)
        # TODO: short circuit the for loop, minimum number of updates?

        # iterate through all pairs and determine which agents can see which targets
        for c in range(len(pairs)):
            if pairs[c][2] > self.agents[pairs[c][0]].get_fov_radius():
                continue
            if self.agents[pairs[c][0]].is_visible(pairs[c][1].get_position()):
                updates[pairs[c][0]].append(pairs[c][1])

        # update the trackers of all agents
        for k in updates:
            self.agents[k].new_detection_set(frame_id, updates[k])

    
    def visible_vertical_targets(self):
        """
        Convenience function for testing targets of single agent
        """
        frame_id = "frame_" + str(self.counter)
        self.counter += 1
        updates = {}
        for k in self.agents:
            updates[k] = []

        # add targets to all agent updates
        for i, t in enumerate(self.targets):
            for k in updates:
                updates[k].append(self.targets[i])

        # update all agents
        for k in updates:
            detections = self.agents[k].create_pov_detection_set_from_targets(
                frame_id, updates[k]
            )
            self.agents[k].load_detection_layer(detections)

    def add_target(self, T):
        """
        Add a target to the world
        """
        self.targets.append(T)


    def get_agent(self,_id):
        """
        Accessor for an agent by id
        """
        return self.agents[_id]

    def check_grid_visibility(self, agent_id, global_target_posn):
        """
        Hidden simulation visibility check 
        """
        GLOBAL_ORIGIN = Position(0,0,0)
        new_posn = self.convert_to_agent_coordinates(agent_id, global_target_posn)
        theta, radius = mfn.car2pol(GLOBAL_ORIGIN.get_cartesian_coordinates(), new_posn.get_cartesian_coordinates())
        phi = 0
        
        sensing_agent = self.get_agent(agent_id)
        
        if sensing_agent.is_visible_fov(theta, phi, radius):
            return True
        return False

    def create_agent_yolobox(self, agent_id, global_target_posn):
        """
        Hidden function for generating valid detections
        """
        DEFAULT_CLASS = 0
        DEFAULT_WIDTH=1
        DEFAULT_HEIGHT=1

        agent_target_posn = self.convert_to_agent_coordinates(agent_id, global_target_posn)
        sensor_target_posn = self.convert_to_sensor_frame_coordinates(agent_id, agent_target_posn)
        
        x,y,z = sensor_target_posn.get_cartesian_coordinates()
        yb = sann.register_annotation(DEFAULT_CLASS,[y,z,DEFAULT_WIDTH,DEFAULT_HEIGHT],distance=x)
        return yb

    def generate_yolo_bbox(self):
        """
        Hidden function for populating yolobox data
        """
        DEFAULT_CLASS = 0
        DEFAULT_WIDTH=1
        DEFAULT_HEIGHT=1
        pass

    def convert_to_agent_coordinates(self, agent_id, target_posn):
        """
        Hidden simulation coordinate transformation
        """
        GLOBAL_ORIGIN = Position(0,0,0)
        sensing_agent = self.get_agent(agent_id)
        theta, radius = mfn.car2pol(sensing_agent.get_origin().get_cartesian_coordinates(), target_posn.get_cartesian_coordinates())
        x1,y1,z1 = mfn.pol2car(GLOBAL_ORIGIN.get_cartesian_coordinates(), radius, theta)
        new_posn = Position(x1,y1,z1)
        return new_posn

    def convert_to_sensor_frame_coordinates(self, agent_id, agent_target_posn):
        """
        Hidden function for converting to an agent's SENSOR frame of reference
        """
        GLOBAL_ORIGIN = Position(0,0,0)

        sa = self.get_agent(agent_id)

        theta, radius = mfn.car2pol(GLOBAL_ORIGIN.get_cartesian_coordinates(), agent_target_posn.get_cartesian_coordinates())
        phi = 0

        x = radius
        y = theta / sa.get_fov_width() * sa.get_max_x() + sa.get_max_x() / 2
        z = phi / sa.get_fov_height() * sa.get_max_y() + sa.get_max_y() / 2
        new_posn = Position(x,y,z)
        return new_posn
