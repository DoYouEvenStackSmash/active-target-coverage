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

  def get_agent(self):
    return (self.exoskeleton, self.sensor)
  
  def get_last_detection(self):
    last_detected, next_detected = self.estimate_rel_next_detection()
    theta = 0
    if len(next_detected):
      theta, r = tfn.calculate_rotation(self.exoskeleton.get_rotation_center(), last_detected, next_detected)
    return theta

  def move_to_next(self):
    theta = self.get_last_detection()
    if theta == 0:
      return
    self.rotate_agent(theta)
    self.obj_tracker.add_angular_displacement((0,(theta, 0)))


  def rotate_agent(self, theta):
    p = self.exoskeleton.get_rotation_center()
    new_theta = self.exoskeleton.rotate_body(self.exoskeleton.get_rotation_center(), theta)
    rot_mat = tfn.calculate_rotation_matrix(new_theta,1)
    self.sensor.origin = tfn.rotate_point(p, self.sensor.origin, rot_mat)
    self.sensor.fov_theta += new_theta
  
  def get_center(self):
    return self.exoskeleton.get_rotation_center()
  
  def get_sensor_origin(self):
    return self.sensor.origin

  def query_sensor_visible(self, target):
    return self.sensor.is_visible(target)

  def get_sensor_range(self):
    return self.sensor.fov_radius

  def query_tracker_for_prediction(self):
    c,pred = self.estimate_next_detection()
    return (c,pred)

  def export_tracks(self):
    '''
    Exports the recorded tracks from the agent's object tracker
    returns a LOCO formatted json object
    '''
    self.obj_tracker.close_all_tracks()
    self.obj_tracker.link_all_tracks(0)
    e = self.obj_tracker.export_loco_fmt()
    return e

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
      last_pt = self.sensor.transform_from_local_coord(lx,ly)
      pred_pt = self.sensor.transform_from_local_coord(px,py)
    
    nd = (last_pt,pred_pt)
    
    return nd
  