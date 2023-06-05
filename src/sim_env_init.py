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
from SimulationEnvironment import SimulationEnvironment
from StreamingObjectTrackManager import ObjectTrackManager
from Detection import *


def init_detection(origin=Position(0, 0, 0), attributes=None):
    """
    Initializes a detection with attributes
    """
    det = Detection(position, attributes)
    return det


def init_agent_exoskeleton(origin=Position(0, 0, 0), sensing_agent=None):
    """
    Initializes an exoskeleton for an agent
    returns a rigid body
    """
    ox, oy, oz = origin.get_cartesian_coordinates()
    scale = 2
    opts = [
        (ox - 10 * scale, oy - 10 * scale, oz),
        (ox - 10 * scale, oy + 10 * scale, oz),
        (ox + 30 * scale, oy, oz),
    ]
    # print(opts)

    mpt = gfn.get_midpoint(opts[0], opts[1])
    mp_origin = Position()
    mp_origin.set_by_triple(mpt)

    print(mpt)
    mpt2 = gfn.get_midpoint(mpt, opts[2])
    mp_center = Position()
    mp_center.set_by_triple(mpt2)

    ap = Polygon(opts)

    mp_endpoint = Position()
    mp_endpoint.set_by_triple(opts[2])

    rb = RigidBody(
        parent_agent=None,
        ref_origin=mp_origin,
        ref_center=mp_center,
        endpoint=mp_endpoint,
        rigid_link=ap,
        states=[],
    )
    if sensing_agent != None:
        rb.parent_agent = sensing_agent
        sensing_agent.exoskeleton = rb
    return rb


def init_sensor(
    width=np.pi / 2,
    height=np.pi / 2,
    radius=200,
    max_x=100,
    max_y=100,
    sensing_agent=None,
):
    """
    Initializes a sensor for a sensing agent
    returns a sensor
    """
    sensor = Sensor(
        sensor_width=width,
        sensor_height=height,
        sensor_radius=radius,
        max_x=max_x,
        max_y=max_y,
    )
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


def init_sensing_agent(
    _id=0,
    origin=Position(0, 0, 0),
    width=np.pi / 2,
    height=np.pi / 2,
    max_x=100,
    max_y=100,
    radius=200,
):
    """
    Standard initializer for sensing agent
    returns a sensing agent
    """
    sensing_agent = SensingAgent()
    exoskeleton = init_agent_exoskeleton(origin, sensing_agent)
    sensor = init_sensor(width, height, radius, max_x, max_y, sensing_agent)
    obj_tracker = init_object_tracker(sensing_agent)
    return sensing_agent


def init_target(origin=Position(0, 0, 0), color=(255, 255, 255), _id=0, path=None):
    """
    standard initializer for target
    returns a Target
    """
    target = Target(origin, color, _id, path)
    return target


def init_sim_environment(
    world_origin=Position(0, 0, 0), sensing_agents=None, targets=None
):
    """
    standard initializer for Environment
    returns an environment
    """
    sensing_agents = sensing_agents if sensing_agents != None else {}
    targets = targets if targets != None else {}
    sim_env = SimulationEnvironment(
        world_origin=world_origin, agents=sensing_agents, targets=targets
    )
    return sim_env
