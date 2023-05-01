#!/usr/bin/python3


from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from StreamingObjectTrackManager import ObjectTrackManager
import numpy as np
  
def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta

class Target:
  def __init__(self, origin, color = None, _id = 0):
    self.origin = origin
    self.color = color
    self._id = _id

  def reposition(self, destination):
    '''
    Repositions the origin of the target to some destination
    '''
    self.origin = destination

  def get_origin(self):
    '''
    Accessor for the origin of the target
    Returns an (x,y) point
    '''
    return self.origin

  def get_id(self):
    '''
    Accessor for the unique identifier of the object
    Returns an id
    '''
    return self._id

class Agent:
  def __init__(self, origin = [0, 0], fov = [0, 100, np.pi / 2], _id = 0, obj_tracker = None):
    self.origin = origin
    self.fov_theta = fov[0]
    self.fov_radius = fov[1]
    self.fov_width = fov[2]
    self._id = _id
    self.obj_tracker = obj_tracker
    self.color = None
  
  def rotate(self, target):
    '''
    Rotates the sensor on the object tracker
    '''
    self.fov_theta = mfn.car2pol(self.origin, target)[0]

  def get_polygon(self):  
    '''
    Gets the convex hull containing the agent origin and the boundary points of its sensor FOV
    returns a list of points
    '''
    adj_point = lambda origin, theta, radius, theta_offset: mfn.pol2car(origin, radius, theta + theta_offset)
    fov_offts = []
    
    fov_offts = [ adjust_angle(self.fov_theta + self.fov_width / 2), 
                  adjust_angle(self.fov_theta + self.fov_width / 4),
                  adjust_angle(self.fov_theta),
                  adjust_angle(self.fov_theta - self.fov_width / 4),
                  adjust_angle(self.fov_theta - self.fov_width / 2) 
                ]

    polygon = [self.origin]
    for i in fov_offts:
      polygon.append(mfn.pol2car(self.origin, self.fov_radius, i))
    return polygon

  def compute_detection_coords(self, target):
    '''
    Calculates detection coordinates relative to Agent
    returns a Yolo Formatted bbox
    '''
    tar_theta, tar_r = mfn.car2pol(self.origin, target)
    tar_theta = mfn.correct_angle(tar_theta)
    org_theta = mfn.correct_angle(self.fov_theta)
    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    
    
    ratio = (lh - tar_theta) / (lh - rh)
    x = 100 * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]
  
  def decompute_detection_coords(self, bbox):
    org_theta = mfn.correct_angle(self.fov_theta)
    rh = org_theta - self.fov_width / 2
    theta = (bbox[0] / 100) * self.fov_width + rh
    theta = adjust_angle(theta)
    r =  bbox[1]
    return mfn.pol2car(self.origin, r, theta)

  

  def is_visible(self, target):
    '''
    Determines whether a target point is in the agent's sensor fov
    Returns true/false
    '''
    tar_theta, tar_r = mfn.car2pol(self.origin, target)
    tar_theta = mfn.correct_angle(tar_theta)
    org_theta = mfn.correct_angle(self.fov_theta)

    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    if tar_r > self.fov_radius:
      return False
    if lh > tar_theta and rh < tar_theta:
      return True
    return False