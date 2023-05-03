#!/usr/bin/python3


from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from StreamingObjectTrackManager import ObjectTrackManager
from Target import Target
import numpy as np
  
def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta


class Agent:
  VALID = 0
  ANGULAR = 1
  RANGE = 2
  TOLERANCE = 0.1
  WINDOW_WIDTH = 100
  def __init__(self, origin = [0, 0], fov = [0, 100, np.pi / 2], _id = 0, obj_tracker = None):
    self.origin = origin
    self.fov_theta = fov[0]
    self.fov_radius = fov[1]
    self.fov_width = fov[2]
    self._id = _id
    self.obj_tracker = obj_tracker
    self.color = None
    self.pose_adjustments = [None]

  def predict_targets_covered(self):
    '''
    Report predicted coverage
    '''
    estimates = []
    if self.obj_tracker.has_active_tracks():
      for t_id in range(len(self.obj_tracker.active_tracks)):
        elem = self.estimate_rel_next_detection(t_id)
        estimates.append(elem)
        
    return estimates
  
  def rotate_sensor(self, target):
    '''
    Rotates the sensor
    Applies a displacement to all active tracks
    Does not return
    '''
    
    theta = mfn.car2pol(self.origin, target)[0]
    
    x,y,w,h = self.transform_to_local_coord(target)
    
    pt = mfn.pol2car(self.origin, y, self.fov_theta)
    x2,y2,w2,h2 = self.transform_to_local_coord(pt)
    target_pt = (x,y)
    pt = (x2,y2)
    
    displacement = (pt, mfn.car2pol(target_pt, pt))
    
    self.obj_tracker.add_angular_displacement(displacement)
    
    self.fov_theta = theta

  def translate_sensor(self, target):
    '''
    Translates the agent based on a sensor reading
    Applies a displacement to all active tracks
    Does not return
    '''
    
    theta, r = mfn.car2pol(self.origin, target)
    x,y,w,h = self.transform_to_local_coord(target)

    x3,y3,w3,h3 = self.transform_to_local_coord(self.origin)
    pt = mfn.pol2car(self.origin,y * 0.1, self.fov_theta)
    
    displacement = (pt, mfn.car2pol((pt[0],pt[1]),self.origin))
    self.obj_tracker.add_linear_displacement(displacement)
    
    self.origin = pt
    
    
  def get_horizontal_axis(self, radius):
    '''
    Helper function for get_sensor_field
    Gets a horizontal line from the agent's fov
    Returns a list of points
    '''
    fov_offts = [ adjust_angle(self.fov_theta + self.fov_width / 2), 
                  adjust_angle(self.fov_theta + self.fov_width / 4),
                  adjust_angle(self.fov_theta),
                  adjust_angle(self.fov_theta - self.fov_width / 4),
                  adjust_angle(self.fov_theta - self.fov_width / 2) 
                ]
    horizontal_axis = []

    for i in fov_offts:
      horizontal_axis.append(mfn.pol2car(self.origin, radius, i))
    return horizontal_axis
  
  def get_sensor_field(self, levels):
    '''
    Gets the coordinate frame of the sensor
    Returns a list of lists of points [[(x1,y1),(x2,y2),...],[...],...]
    '''
    y_step = self.fov_radius / levels
    axes = []
    for i in range(1, levels + 1):
      axes.append(self.get_horizontal_axis(y_step * i))
    return axes

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
  
    org_theta = self.fov_theta
    if org_theta < 0:
      org_theta = 2 * np.pi + org_theta
 
    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    if tar_theta < rh:
      tar_theta = tar_theta + 2 * np.pi
    
    ratio = (lh - tar_theta) / (lh - rh)
    
    x = Agent.WINDOW_WIDTH * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]
  
  
  def transform_from_local_coord(self, x, y, w=1, h=1):
    '''
    Transforms a bbox from agent local coordinates to world coordinates
    returns a Point
    '''
    org_theta = mfn.correct_angle(self.fov_theta)
    
    rh = org_theta - self.fov_width / 2
    lh = org_theta + self.fov_width / 2
    theta = (x / Agent.WINDOW_WIDTH) * self.fov_width
    # print(f"is_not_visible: {lh}:{theta}:{rh}")
    ratio = theta - 0.5
    theta = lh - theta
    theta = mfn.correct_angle(theta)
    
    r =  y
    return mfn.pol2car(self.origin, r, theta)

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
  
  def get_predictions(self):
    '''
    Estimates next detections
    Returns a list of pairs of xy points[((x1,y1),(x2,y2))]
    '''
    predictions = []
    if not self.obj_tracker.has_active_tracks():
      return []
    for i in range(len(self.obj_tracker.active_tracks)):
      predictions.append(self.estimate_next_detection(i))
    return predictions

  def is_detectable(self, target):
    '''
    Indicates whether a target point is detectable (within tolerance)
    Returns a boolean indicator and a type identifier
    '''
    adj_win_bnd = Agent.WINDOW_WIDTH * Agent.TOLERANCE  
    adj_rad_bnd = self.fov_radius * (Agent.TOLERANCE/1)

    if target[0] < 0 + adj_win_bnd or target[0] > Agent.WINDOW_WIDTH - adj_win_bnd:
      return False, Agent.ANGULAR

    if target[1] > self.fov_radius - adj_rad_bnd  or target[1] < 0 + adj_rad_bnd:
      return False, Agent.RANGE
    
    return True, Agent.VALID

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
    
    return False
  
  def adjust_for_coverage(self):
    '''
    Adjusts agent pose to improve coverage of predictions
    Does not return
    '''

    estimates = self.predict_targets_covered()
    j = 0
    while j < len(estimates):
      i = estimates[j]
      indicator, err_type = self.is_detectable(i[1])
      if indicator:
        j+=1
        continue
      else:
        pred = self.transform_from_local_coord(i[1][0],i[1][1])
        curr = self.transform_from_local_coord(i[0][0],i[0][1])
        rotation = gfn.lerp_list(curr, pred, 2)
        start = 1
          
          # incrementally rotate the agent
        for p in rotation[start:]:
          if len(pred) > 0:
            if err_type == Agent.ANGULAR:
              self.rotate_sensor(p)
            elif err_type == Agent.RANGE:
              self.translate_sensor(p)
          
        estimates = self.predict_targets_covered()
        j = 0
        break

  def export_tracks(self):
    '''
    Exports the recorded tracks from the agent's object tracker
    returns a LOCO formatted json object
    '''
    self.obj_tracker.close_all_tracks()
    self.obj_tracker.link_all_tracks(0)
    e = self.obj_tracker.export_loco_fmt()
    return e
    