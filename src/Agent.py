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

  def transform_to_local_coord(self, target):
    '''
    Calculates detection coordinates relative to Agent
    returns a Yolo Formatted bbox
    '''

    tar_theta, tar_r = mfn.car2pol(self.origin, target)
    tar_theta = mfn.correct_angle(tar_theta)
    org_theta = mfn.correct_angle(self.fov_theta)

    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    
    # ratio = abs(org_theta - tar_theta) / self.fov_width
    ratio = (lh - tar_theta) / (self.fov_width)
    # (lh - tar_theta) / (self.fov_width)
    x = 100 * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]
  
  
  def transform_from_local_coord(self, bbox):
    '''
    Transforms a bbox from agent local coordinates to world coordinates
    returns a Point
    '''
    org_theta = mfn.correct_angle(self.fov_theta)
    
    rh = org_theta - self.fov_width / 2
    lh = org_theta + self.fov_width / 2
    theta = (bbox[0] / 100) * self.fov_width
    print(f"is_not_visible: {lh}:{theta}:{rh}")
    ratio = theta - 0.5
    theta = lh - theta
    theta = mfn.correct_angle(theta)
    # theta = rh + theta
     
    # print(f"is_not_visible: {lh}:{tar_theta}:{rh}")
    # ltheta = lh - theta
    # rtheta = rh + theta

    # theta = adjust_angle(theta)
    
    r =  bbox[1]
    return mfn.pol2car(self.origin, r, theta)

  def estimate_next_detection(self):
    '''
    U
    '''
    if self.obj_tracker.active_tracks != None and len(self.obj_tracker.active_tracks):
      trk = self.obj_tracker.active_tracks[0]
      pred_pt = trk.predict_next_box()
      x,y = pred_pt
      pred_pt = self.transform_from_local_coord([x,y,1,1])
      x,y = pred_pt
      return (x,y)
    else:
      print("cannot estimate next detection!")
      return ()
    

  def is_visible(self, target):
    '''
    Determines whether a target point is in the agent's sensor fov
    Returns true/false
    '''
    tar_theta, tar_r = mfn.car2pol(self.origin, target)
    
    tar_theta = adjust_angle(tar_theta)
    
    org_theta = mfn.correct_angle(self.fov_theta)

    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    if tar_theta < 0 and lh > 0 and rh > 0:
      tar_theta = 2 * np.pi + tar_theta
    if tar_r > self.fov_radius:
      return False
    if lh > tar_theta and rh < tar_theta:
      return True
    print(f"is_not_visible: {lh}:{tar_theta}:{rh}")
    return False