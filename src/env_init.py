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
from Environment import Environment
from StreamingObjectTrackManager import ObjectTrackManager


def init_agent_exoskeleton(origin=(0, 0), sensing_agent=None):
    """
    Initializes an exoskeleton for an agent
    returns a rigid body
    """
    ox, oy = origin
    scale = 1.5
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


def init_target(origin=(0, 0), color=(255, 255, 255), _id=0, path=None):
    """
    standard initializer for target
    returns a Target
    """
    target = Target(origin, color, _id, path)
    return target


def init_environment(world_origin=(0, 0), sensing_agents=None, targets=None):
    """
    standard initializer for Environment
    returns an environment
    """
    sensing_agents = sensing_agents if sensing_agents != None else {}
    targets = targets if targets != None else {}
    env = Environment(world_origin=world_origin, agents=sensing_agents, targets=targets)
    return env
