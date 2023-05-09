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
    return self.obj_tracker

  def translate_agent(self, target_pt):
    '''
    Wrapper for translating the rigid body of the agent
    returns a displacement vector (theta, r)
    '''
    theta, r = self.exoskeleton.translate_body(target_pt)
    return (theta, r)

  def rotate_agent(self, target_pt, center_line = None):
    '''
    Wrapper for rotating the rigid body of the agent
    returns an angle theta
    '''
    rotation = 0
    rotation = self.exoskeleton.rotate_body(target_pt)
    return rotation

  def apply_rotation_to_agent(self, rotation):
    '''
    Applies a rotation in radians to the exoskeleton
    returns the angle in radians 
    '''
    rotation = self.exoskeleton.apply_rotation_to_body(rotation)
    # self.exoskeleton.rel_theta += rotation
    return rotation

  def apply_translation_to_agent(self, translation_dist):
    '''
    Applies a translation as a vector to the exoskeleton
    returns a vector
    '''
    translation_dist = self.exoskeleton.apply_translation_to_body(translation_dist)
    return translation_dist

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

  def is_rel_detectable(self, target_pt):
    '''
    Indicates whether a target point is detectable (within tolerance)
    Returns a boolean indicator and a type identifier
    '''
    # boundary conditions
    adj_win_bnd = (self.get_fov_width() / 2) - (self.get_fov_width() / 2) * Sensor.TOLERANCE * 2
    adj_rad_bnd = self.get_fov_radius()

    target_x = target_pt[0]
    target_y = target_pt[1]
    flags = 0
    
    angle_flag = False
    
    # if range out of bounds
    if target_y > adj_rad_bnd - adj_rad_bnd * Sensor.TOLERANCE or target_y < 0 + adj_rad_bnd * Sensor.TOLERANCE:
      flags += Sensor.RANGE

    # if angle out of bounds
    if target_x < 0 + Sensor.WINDOW_WIDTH * Sensor.TOLERANCE or target_x > Sensor.WINDOW_WIDTH - Sensor.WINDOW_WIDTH * Sensor.TOLERANCE:
      flags += Sensor.ANGULAR
    if flags > 0:
      return False, flags

    return True, Sensor.VALID
  
  def export_tracks(self):
    '''
    Exports the recorded tracks from the agent's object tracker
    returns a LOCO formatted json object
    '''
    self.obj_tracker.close_all_tracks()
    self.obj_tracker.link_all_tracks(0)
    e = self.obj_tracker.export_loco_fmt()
    return e

  def new_detection_layer(self,frame_id,add_list):
    '''
    Ingest for a new layer of detections from the outside world
    '''
    detections = []
    for a in add_list:
      dc = self.transform_to_local_bbox(a.get_origin())
      yb = sann.register_annotation(0, dc, frame_id)
      detections.append(yb)
    self.obj_tracker.add_new_layer(detections)
    self.obj_tracker.process_layer(len(self.obj_tracker.layers) - 1)

  def transform_to_local_bbox(self,target_pt):
    '''
    Calculates detection coordinates relative to Sensor
    returns a Yolo Formatted bbox
    '''
    target_rotation = self.exoskeleton.get_relative_rotation(target_pt)
    ratio = (target_rotation / self.get_fov_width())

    r = mfn.euclidean_dist(self.get_center(), target_pt)

    x = Sensor.WINDOW_WIDTH * ratio + 50
    y = r
    w = 1
    h = 1
    
    return [x,y,w,h]

  def transform_from_local_coord(self, x, y, w=1, h=1):
    '''
    Transforms a bbox from sensor local coords to world coords
    returns a point
    '''
    theta = (x - 50) / Sensor.WINDOW_WIDTH * self.get_fov_width()
    theta = adjust_angle(self.get_fov_theta() + theta)
    r = y
    pt = mfn.pol2car(self.get_center(), r, theta)

    return pt

  def estimate_next_rotation(self, idx = 0):
    '''
    Uses past information to predict the next rotation if it exists
    Returns () or the tuple containing the partial rotation
    '''
    curr_pt, pred_pt = self.estimate_rel_next_detection()
    
    # if no estimate available
    if not len(pred_pt):
      return (None,None)
    
    # if first element in track, therefore duplicate
    if curr_pt == pred_pt:
      return (None,None)
    
    status, flag = self.is_rel_detectable(pred_pt)
    
    # if predicted point is detectable from pov of SensingAgent
    if status:
      return (None,None)
    
    # if predicted point is out of coverage by range
    if flag == Sensor.RANGE:
      offset = pred_pt[1] - (self.get_fov_radius() * (1 - Sensor.TOLERANCE))
      if pred_pt[1] < self.get_fov_radius() * Sensor.TOLERANCE:
        offset = pred_pt[1] - self.get_fov_radius() * Sensor.TOLERANCE
      return (None, offset)
    
    # if predicted point is out of coverage by angle
    if flag == Sensor.ANGULAR:
      partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
      return (partial_rotation,None)
    
    # if predicted point is out of coverage by both angle and range
    if flag == Sensor.BOTH:
      offset = pred_pt[1] - (self.get_fov_radius() * (1 - Sensor.TOLERANCE))
      if pred_pt[1] < self.get_fov_radius() * Sensor.TOLERANCE:
        offset = pred_pt[1] - self.get_fov_radius() * Sensor.TOLERANCE
      partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
      return (partial_rotation, offset)


  
  def estimate_rel_next_detection(self, idx = 0):
    '''
    Estimates next detection in local coordinate system
    returns a pair of points
    '''
    last_pt, pred_pt = (),()
    
    if self.obj_tracker.has_active_tracks():
      trk = self.obj_tracker.active_tracks[idx]
      trk_h = trk.get_track_heading()
      last_pt = trk.get_last_detection()
      pred_pt = trk.predict_next_box()
    
    nd = (last_pt, pred_pt)

    return nd

  def estimate_next_detection(self, idx = 0):
    '''
    Estimates next detection in external coordinate system
    returns a pair of points
    '''
    last_pt,pred_pt = (),()
    nd = self.estimate_rel_next_detection(idx)
    if len(nd[0]) and len(nd[1]):
      lx,ly = nd[0]
      px,py = nd[1]
      last_pt = self.transform_from_local_coord(lx,ly)
      pred_pt = self.transform_from_local_coord(px,py)
    
    nd = (last_pt,pred_pt)
    
    return nd