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
    rotation = self.exoskeleton.apply_rotation_to_body(rotation)
    # self.exoskeleton.rel_theta += rotation
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

  def predict(self):
    curr_pt, pred_pt = self.estimate_rel_next_detection()
    if not len(pred_pt):
      return
    status, flag = self.is_rel_detectable(pred_pt)
    if status:
      print(status)
      return
    if flag == Sensor.ANGULAR:
      if curr_pt == pred_pt:
        return
      print(f"curr: {curr_pt}")
      print(f"pred: {pred_pt}")
      # pred_pt = [pred_pt[0], pred_pt[1]]
      
      p_pt = (pred_pt[0] - 50, pred_pt[1])
      c_pt = (curr_pt[0] - 50, curr_pt[1])
      theta, r = mfn.car2pol((0,0), p_pt)
      theta2, r2 = mfn.car2pol((0,0), c_pt)
      distance = mfn.car2pol(p_pt, c_pt)[1]
      ref_theta, ref_r = mfn.car2pol((0,0), (0,50))
      # print(theta - ref_theta)
      rotation = ref_theta - theta
      print(rotation)
      print("rotating")
      rotation = self.apply_rotation_to_agent(rotation)
      self.obj_tracker.add_angular_displacement(((0,0), (-rotation, distance)))
      self.exoskeleton.rel_theta += rotation
      # print(theta - theta2)
      # print(theta2)
      
      


  def is_rel_detectable(self, target_pt):
    '''
    Indicates whether a target point is detectable (within tolerance)
    Returns a boolean indicator and a type identifier
    '''
    adj_win_bnd = (self.get_fov_width() / 2) - (self.get_fov_width() / 2) * Sensor.TOLERANCE * 2
    adj_rad_bnd = self.get_fov_radius()

    target_x = target_pt[0]
    target_y = target_pt[1]

    if target_y > adj_rad_bnd - adj_rad_bnd * Sensor.TOLERANCE or target_y < 0 + adj_rad_bnd * Sensor.TOLERANCE:
      return False, Sensor.RANGE

    if target_x < 0 + Sensor.WINDOW_WIDTH * Sensor.TOLERANCE or target_x > Sensor.WINDOW_WIDTH - Sensor.WINDOW_WIDTH * Sensor.TOLERANCE:
      return False, Sensor.ANGULAR
    
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
  
  def _transform_to_local_bbox(self, target):
    '''
    Calculates detection coordinates relative to Agent
    returns a Yolo Formatted bbox
    '''
    tar_theta, tar_r = mfn.car2pol(self.get_center(), target)
  
    org_theta = self.get_fov_theta()
    if org_theta < 0:
      org_theta = 2 * np.pi + org_theta

    lh = org_theta + self.get_fov_width() / 2
    rh = org_theta - self.get_fov_width() / 2
    if tar_theta < rh:
      tar_theta = tar_theta + 2 * np.pi
    
    ratio = (lh - tar_theta) / (lh - rh)
    
    x = Sensor.WINDOW_WIDTH * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]

  
  def transform_from_local_coord(self, x, y, w=1, h=1):
    '''
    Transforms a bbox from sensor local coords to world coords
    returns a point
    '''
    # theta = x / Sensor.WINDOW_WIDTH * self.get_fov_width()
    theta, r = mfn.car2pol((50,0), (x,y))
    # print(f"pt{(x,y)}")
    
    origin = self.get_center()
    angle = self.get_fov_theta()
    new_angle = self.get_fov_theta() + theta
    if (new_angle > np.pi):
      new_angle = -2 * np.pi + new_angle
    if new_angle < -np.pi:
      new_angle = 2 * np.pi + new_angle

    # m_theta = self.get_fov_theta()
    
    # m_center = self.get_center()
    
    pt = mfn.pol2car(self.get_center(), r, new_angle)
    return pt
  
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