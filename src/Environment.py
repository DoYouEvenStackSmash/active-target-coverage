#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn

import numpy as np
from support.Polygon import Polygon
from Sensor import Sensor
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Target import Target
import json
from StreamingObjectTrackManager import ObjectTrackManager

# from env_init import init_sensing_agent


def init_agent_exoskeleton(origin=(0, 0), sensing_agent=None):
    """
    Initializes an exoskeleton for an agent
    returns a rigid body
    """
    ox, oy = origin
    scale = 0.2
    # makes the agent look like a triangle
    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
    # makes the agent look like an hourglass thing
    scale2 = 1.5
    opts2 = [
        (ox - 10 * scale2, oy - 10 * scale2),
        (ox - 10 * scale2, oy + 10 * scale2),
        (ox + 20 * scale2, oy - 10 * scale2),
        (ox + 20 * scale2, oy + 10 * scale2),
        (ox - 10 * scale2, oy - 10 * scale2),
        (ox - 10 * scale2, oy + 10 * scale2),
        (ox + 20 * scale2, oy + 10 * scale2),
        (ox + 20 * scale2, oy - 10 * scale2),
        # (ox + 30 * scale, oy),
    ]

    # print(opts)

    mpt = gfn.get_midpoint(opts[0], opts[1])
    mpt2 = gfn.get_midpoint(mpt, opts[2])

    ap = Polygon(opts)

    rb = RigidBody(
        parent_agent=None,
        ref_origin=mpt,
        ref_center=mpt2,
        endpoint=opts[2],
        rigid_link=ap,
        states=[],
    )
    if sensing_agent != None:
        rb.parent_agent = sensing_agent
        sensing_agent.exoskeleton = rb
    return rb


def init_sensor(width=np.pi / 2, radius=200, sensing_agent=None):
    """
    Initializes a sensor for a sensing agent
    returns a sensor
    """
    sensor = Sensor(sensor_width=width, sensor_radius=radius)
    if sensing_agent != None:
        sensing_agent.centered_sensor = sensor
        sensor.parent_agent = sensing_agent
    return sensor


def init_object_tracker(sensing_agent=None):
    """
    Initializes an object tracker for a sensing agent
    returns an object track manager
    """
    obj_tracker = ObjectTrackManager()
    if sensing_agent != None:
        obj_tracker.parent_agent = sensing_agent
        sensing_agent.obj_tracker = obj_tracker
    return obj_tracker


def init_sensing_agent(_id=0, origin=(0, 0), width=np.pi / 2, radius=200):
    """
    Standard initializer for sensing agent
    returns a sensing agent
    """
    sensing_agent = SensingAgent()
    exoskeleton = init_agent_exoskeleton(origin, sensing_agent)
    sensor = init_sensor(width, radius, sensing_agent)
    obj_tracker = init_object_tracker(sensing_agent)
    return sensing_agent


def init_target(
    origin=(0, 0), color=(255, 255, 255), _id=0, path=None, attributes=None
):
    """
    standard initializer for target
    returns a Target
    """
    target = Target(origin, color, _id, path, attributes)
    return target


# import sys

# from RigidBody import RigidBody
# from Sensor import Sensor
# from SensingAgent import SensingAgent
# import pygame
# import time
# from env_init import *


class Environment:
    """A class for simulating the environment in which the agents operate
    Attributes:
        Agent (Agent): legacy Agent attribute
        Agents (List[Agent]): list of all agents present in the environment
        Targets (List[Target]): list of all targets present in the environment

    Mock environment for simulating the world
    """

    def __init__(
        self,
        world_origin=(0, 0),
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
        frame_id = self.counter

        self.counter += 1
        updates = {}

        for k in self.agents:
            if self.agents[k].IS_DEAD:
                continue
            # self.agents[k].heartbeat()
            updates[k] = []
            for target in self.targets:
                d = mfn.euclidean_dist(
                    self.agents[k].get_origin(), target.get_position()
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

    def target_coverage(self, frame_id, measurement_rate):
        uncovered_targets = len(self.targets)
        pairs = []
        sortkey = lambda x: x[2]
        
        # min_dist
        # frame_id = "frame_" + str(self.counter)
        self.counter += 1
        updates = {}
        for k in self.agents:
            if self.agents[k].IS_DEAD:
                continue
            # self.agents[k].heartbeat()
            updates[k] = []
            pts = self.agents[k].estimate_next_detection()
            pt = None
            if not len(pts):
              pt = self.agents[k].get_origin()
            else:
                pt = pts[0][1]
            for target in self.targets:
                # avg_dist = 
                
                d = mfn.euclidean_dist(
                    pt, target.get_position()
                )
                
                pairs.append((self.agents[k]._id, target, d))

        pairs = sorted(pairs, key=sortkey)
        # TODO: short circuit the for loop, minimum number of updates?
        covered_targets = set()
        busy_agents = set()
        # iterate through all pairs and determine which agents can see which targets
        for c in range(len(pairs)):
            if pairs[c][1].get_id() in covered_targets:
                continue
            if pairs[c][0] in busy_agents:
                continue
            if pairs[c][2] > self.agents[pairs[c][0]].get_fov_radius():
                continue
            if self.agents[pairs[c][0]].is_visible(pairs[c][1].get_position()):
                updates[pairs[c][0]].append(pairs[c][1])
                covered_targets.add(pairs[c][1].get_id())
                uncovered_targets -= 1
                busy_agents.add(self.agents[pairs[c][0]]._id)
        
        unused_agents = []
        for ua in self.agents:
            ba = self.agents[ua]
            if ba.IS_DEAD:
                continue
            if ba._id not in busy_agents:
                unused_agents.append(ba._id)
        pairs = []
        for k in unused_agents:
            updates[k] = []

            # pt = mfn.pol2car(self.agents[k].get_origin(), self.agents[k].get_fov_radius() / 2, self.agents[k].get_fov_theta())

            for t in self.targets:
                if t.get_id() in covered_targets:
                    continue
                d = mfn.euclidean_dist(
                    self.agents[k].get_origin(), target.get_position()
                )
                pairs.append((self.agents[k]._id, target, d))
        pairs = sorted(pairs, key=sortkey)

        for c in range(len(pairs)):
            if pairs[c][1].get_id() in covered_targets:
                continue
            if pairs[c][0] in busy_agents:
                continue
            if pairs[c][2] < self.agents[pairs[c][0]].get_fov_radius():
                self.agents[pairs[c][0]].rotate_agent(pairs[c][1].get_origin())
                updates[pairs[c][0]].append(pairs[c][1])
                covered_targets.add(pairs[c][1].get_id())
                uncovered_targets -= 1
                busy_agents.add(self.agents[pairs[c][0]]._id)
                continue
            
            else:
                k = pairs[c][0]
                theta, radius = mfn.car2pol(
                    self.agents[k].get_origin(), pairs[c][1].get_origin()
                )
                pt = mfn.pol2car(
                    self.agents[k].get_origin(),
                    radius - self.agents[k].get_fov_radius() / 2,
                    theta,
                )
                self.agents[k].rotate_agent(pt)
                self.agents[k].translate_agent(pt)

                # self.agents[k].obj_tracker.link_all_tracks()
                self.agents[k].obj_tracker.close_all_tracks()
                self.agents[k].clock = 0
                # self.agents[k].heartbeat()
                updates[k].append(pairs[c][1])
                covered_targets.add(pairs[c][1].get_id())
                uncovered_targets -= 1
                busy_agents.add(k)

        for t in self.targets:
            if t.get_id() in covered_targets:
                continue
            x, y = t.get_origin()
            print(x,y)
            var = measurement_rate * 60
            sa = init_sensing_agent(
                origin=(x + var / 2, y + var / 2), width=np.pi, radius=var
            )
            sa.obj_tracker.radial_exclusion = 600
            sa.centered_sensor.tolerance = 0.45
            sa.obj_tracker.avg_window_len = int(10 * (1 / measurement_rate))
            sa.obj_tracker.track_lifespan = 10
            sa._id = len(self.agents)
            sa.rotate_agent((x, y))
            sa.heartbeat()
            updates[sa._id] = []
            updates[sa._id].append(t)
            self.agents[sa._id] = sa
            uncovered_targets -= 1
            covered_targets.add(t.get_id())

        for k in updates:
            self.agents[k].new_detection_set(frame_id, updates[k])

    def add_target(self, T):
        """
        Add a target to the world
        """
        self.targets.append(T)

    def transform_from_local_coord(self, x, y, w=1, h=1):
        """
        Transforms a bbox from Sensor local coordinates to world coordinates
        returns a Point
        Args:
            x(float) : x coordinate
            y(float) : y coordinate
            w(float) : width of the bounding box
            h(float) : height of the bounding box
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

    def serialize_agent_tracks(self):
        """
        export tracks of all agents
        """
        for k in self.agents:
            sensing_agent = self.agents[k]
            e = sensing_agent.export_tracks()
            f = open(f"{sensing_agent._id}_out.json", "w")

            f.write(json.dumps(e, indent=2))
            f.close()
