#!/usr/bin/python3
import numpy as np
from aux_functions import *

class ObjectTrack:
  def __init__(self, track_id, class_id):
    self.r = 0
    self.theta = 0
    self.delta_theta = []
    self.delta_v = []
    self.path = []
    self.track_id = track_id
    self.color = rand_color()
    self.last_frame = -1
    self.class_id = class_id

  def add_new_step(self, yb, frame_id):
    ''' 
    Add a new bounding box to the object track
    '''
    # update velocity  
    if len(self.path) > 0:
      self.update_track_vector(yb.get_center_coord())
  
    self.last_frame = frame_id
    yb.parent_track = self.track_id
    self.path.append(yb)

  
  def update_track_vector(self, pt):
    '''
    Update track velocity vector
    Assumes path is not empty
    '''
    normalize_theta = lambda theta: theta if theta > 0 else 2 * np.pi + theta
    cx,cy = pt
    lx,ly = self.path[-1].get_center_coord()
    r = MathFxns.euclidean_dist((lx,ly),(cx,cy))
    self.delta_v.append(r / self.r)
    self.r = r

    theta = np.arctan2(cx - lx, cy - ly)
    self_theta = normalize_theta(self.theta)
    new_theta = normalize_theta(theta)
    self.delta_theta.append(new_theta / self_theta)
    
    self.theta = theta

    # add recent velocity to delta_v
  
  def predict_next_box(self):
    '''
    Predict next bounding box center
    '''
    lx,ly = self.path[-1].get_center_coord()
    if len(self.path) == 1:
      return (lx,ly)
    return (lx + (self.r * np.cos(self.theta)), ly + (self.r * np.sin(self.theta)))
  
  
  def is_alive(self, fc, expiration):
    '''
    Check whether a track is expired
    '''
    return bool(fc - self.last_frame < expiration)
  
  def reflect_track(self, reflect_axis = None):
    '''
    reflect across an axis
    '''
    if reflect_axis == None:
      return 
    for ybx in self.path:
      if reflect_axis == 1:
        ybx.reflectX()
      elif reflect_axis == 0:
        ybx.reflectY()
      else:
        break
    
  def rotate_track(self, offset_deg):
    ''' 
    rotate all boxes on an object track
    Adjusts rotations to ensure rotation is always 90 degrees
      e.g. 270 -> -90
      180 -> reflectXY()
    '''
    if offset_deg == 0:
      return

    dir_flag = 1
    if abs(offset_deg) == 180:
      for ybx in self.path:
        ybx.reflectXY()
      return
    
    # correct rotations to ensure 90 degrees
    if abs(offset_deg) == 270:
      offset_deg = 90 if offset_deg < 0 else -90
    
    # adjust direction
    if offset_deg < 0:
      dir_flag = -1
    
    # Rotate quadrant of each box
    for ybx in self.path:
      ybx.rotate_quad(dir_flag)
  
  
  def link_path(self):
    '''
    Postprocessing step to construct a linked list
    '''
    for i in range(len(self.path)-1):
      self.path[i].next = self.path[i + 1]
      self.path[i+1].prev = self.path[i]

    
  def get_step_count(self):
    '''
    Accessor for checking length of the track
    '''
    return len(self.path)
  
      
  def get_loco_track(self,fdict = None,steps = []):
    '''
    Get complete track in loco format
    template = {
                "id":counter, 
                "image_id": 0, 
                "category_id":1.0, 
                "bbox" : [
                    cx,
                    cy,
                    w,
                    h
                ], 
                "segmentation": [], 
                "iscrowd": 0, 
                "track_id" : self.track_id,
                "vid_id":0
                }
    '''
    for yb in self.path:
      fid = None
      # if fdict != None:
      #   fid = fdict[f'{yb.img_filename[:-3]}png']
      # else:
      fid = yb.img_filename
      steps.append({
              "id":-1, 
              "image_id": fid, 
              "category_id":yb.class_id, 
              "bbox" : yb.bbox, 
              "area": yb.bbox[2] * yb.bbox[3], 
              "segmentation": [], 
              "iscrowd": 0, 
              "track_id" : self.track_id,
              "trackmap_index" : -1,
              "vid_id":0, 
              "track_color": self.color})
    
      