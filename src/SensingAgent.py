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
# from Scene import *
import json

# import pygame
# import numpy as np
import sys
# import time

from RigidBody import RigidBody
from Sensor import Sensor
# from SensingAgent import SensingAgent
import pygame
import time


def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta

class SensingAgent: 
  def __init__(self,
              exoskeleton = None,
              sensor = None,
              obj_tracker = None,
              _id = None,
              ):
    self.exoskeleton = exoskeleton
    self.sensor = sensor
    self.obj_tracker = obj_tracker
    self._id = _id

  def get_origin(self):
    '''
    Accessor for the origin of the sensing agent in rigid body
    returns an (x,y) point
    '''
    origin = self.exoskeleton.get_center()
    return origin
  
  def get_fov_theta(self):
    '''
    Accessor for the orientation of the sensing agent as the rigid body
    Returns an angle theta
    '''
    fov_theta = self.exoskeleton.get_rel_theta()
    return fov_theta    

  def get_fov_width(self):
    '''
    Accessor for the width of the fov of the sensing agent
    returns a scalar value
    '''
    fov_width = self.sensor.get_fov_width()
    return fov_width

  def get_fov_radius(self):
    '''
    Accessor for the range of the sensing agent's sensor
    returns a scalar value
    '''
    fov_radius = self.sensor.get_fov_radius()
    return fov_radius

  def get_components(self):
    '''
    Returns the attributes of an agent
    '''
    return (self.exoskeleton, self.sensor)

  def get_center(self):
    '''
    Accessor for rotation center of the agent
    '''
    return self.exoskeleton.get_center()
  
  def get_sensor(self):
    '''
    Accessor for the agent's sensor
    '''
    return self.sensor

  def get_object_tracker(self):
    '''
    Accessor for the agent's object tracker
    '''

  def translate_agent(self, target_pt):
    '''
    Wrapper for translating the rigid body of the agent
    returns a displacement vector (theta, r)
    '''
    theta, r = self.exoskeleton.translate_body(target_pt)
    return (theta, r)

  def rotate_agent(self, target_pt):
    '''
    Wrapper for rotating the rigid body of the agent
    returns an angle theta
    '''
    rotation = self.exoskeleton.rotate_body(target_pt)
    return rotation
  
  def is_visible(self, target_pt):
    '''
    Determines whether a target point is in the Sensor's sensor fov
    Returns true/false
    '''
    rotation = self.exoskeleton.get_relative_rotation(target_pt)
    theta, r = mfn.car2pol(self.exoskeleton.get_center(), target_pt)
    if abs(rotation) > self.get_fov_width() / 2:
      return False
    if r > self.get_fov_radius():
      return False
    return True

  def export_tracks(self):
    '''
    Exports the recorded tracks from the agent's object tracker
    returns a LOCO formatted json object
    '''
    self.obj_tracker.close_all_tracks()
    self.obj_tracker.link_all_tracks(0)
    e = self.obj_tracker.export_loco_fmt()
    return e

