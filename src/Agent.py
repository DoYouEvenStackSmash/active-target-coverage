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
  
  def __init__(self, 
              origin = [0, 0],
              orientation = 0,
              sensors = [],
              _id = 0,
              obj_tracker = None,
              body = None):
    
    self.origin = origin
    self.orientation = orientation
    self.sensors = []
    self._id = _id
    self.obj_tracker = obj_tracker
    self.color = None
    self.pose_adjustments = [None]
    self.body = None

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
  
  def rotate_agent_to_target(self, target_pt = None, target_displacement = None, sensor_idx = 0):
    '''
    Adjusts the sensor orientation to point at a target
    Wrapper for calculate rotation and calculate translation

    applies displacement to object tracker
    Does not return
    '''
    rotation = None
    translation = None
    if target_displacement != None:
      rotation = target_displacement[0]
      translation = target_displacement[1]
    elif target_pt != None:
      translation = self.calculate_agent_translation(target_pt)
      rotation = self.calculate_agent_rotation(target_pt)
    else:
      print("no displacement provided!")
      return
    self.sensors[0].adjust_fov(rotation)
    total_rotation = self.chains[0].rotate_single_link(target_pt)

    self.obj_tracker.add_angular_displacement(rotation)
    
    self.chains[0].translate_chain(target_pt)
    
    self.sensors[0].adjust_origin(translation)
    self.obj_tracker.add_linear_displacement(translation)

  def rotate_chain_to_target(self, link_count = 2, target_pt = None, target_displacement = None, Olist = []):
    '''
    Applies a rotation followed by translation according to some displacement or target point
    to all chains.
    
    Does not return
    '''
    total_rotation = 0
    if target_pt == None:
      print("no target point specified")
      return
    if link_count == 2:
      intermediate_p = self.chains[0].preprocess_circles(target_pt)
      end_p = target_pt
      total_rotation = self.chains[0].rotate_two_link_chain(target_pt, intermediate_pt, steps=10, Olist = Olist)
    elif link_count == 1:
      total_rotation = self.chains[0].rotate_single_link(target_pt, Olist=Olist)
    self.obj_tracker.add_angular_displacement(total_rotation)
    self.sensors.adjust_fov(rotation)
      

  def calculate_agent_rotation(self, target, sensor_idx = 0):
    '''
    Calculates rotation adjustment to point at a target
    returns an angle theta
    '''

    theta = mfn.car2pol(self.origin, target)[0]
    
    x,y,w,h = self.transform_to_local_coord(target)
    
    pt = mfn.pol2car(self.origin, y, self.fov_theta)
    x2,y2,w2,h2 = self.transform_to_local_coord(pt)
    target_pt = (x,y)
    pt = (x2,y2)
    
    displacement = (pt, mfn.car2pol(target_pt, pt))
    return displacement

  def calculate_agent_rotation(self, target, chain_idx = 0):
    '''
    calculates rotation adjustment to point at a target
    returns an angle theta
    '''
    theta = mfn.car2pol(self.origin, target)[0]
    
    x,y,w,h = self.transform_to_local_coord(target)
    
    pt = mfn.pol2car(self.origin, y, self.fov_theta)
    x2,y2,w2,h2 = self.transform_to_local_coord(pt)
    target_pt = (x,y)
    pt = (x2,y2)
    
    displacement = (pt, mfn.car2pol(target_pt, pt))
    return displacement
  
  def calculate_agent_translation(self, target):
    '''
    Calculates translation adjustment to move to at a target
    Returns a distance d
    '''
    theta, r = mfn.car2pol(self.origin, target):
    return r
    


  def rotate_sensor(self, target):
    '''
    Rotates the sensor
    Applies a displacement to all active tracks
    Does not return
    '''
    # print(f"angular: {displacement[1]}")
    # print(self.fov_theta, theta)
    distance = self.fov_theta - theta
    if (distance > np.pi):
      distance = 2 * np.pi - distance
    if (distance < -np.pi):
      distance = -2 * np.pi - distance

    self.fov_theta = theta
    return distance

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

    return displacement[1][1]
    
  

  
  def get_sensor_field(self, sensor_idx = 0):
    '''
    Accessor for the field of view for a sensor
    Returns a list of lists of points
    '''
    return self.sensors[sensor_idx]
  

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
              disp = self.rotate_sensor(p)
            # elif err_type == Agent.RANGE:
            #   disp = self.translate_sensor(p)
          
        estimates = self.predict_targets_covered()
        j = 0
        break
    
  def publish_adjustment(self):
    '''
    Exposes a target point for an actor to improve coverage
    '''

    estimates = self.predict_targets_covered()
    if len(estimates) == 0:
      return (), Agent.VALID
    for j in range(len(estimates)):
      i = estimates[j]
      indicator, err_type = self.is_detectable(i[1])
      if indicator:
        j+=1
        continue
      else:
        pred = self.transform_from_local_coord(i[1][0],i[1][1])
        curr = self.transform_from_local_coord(i[0][0],i[0][1])
        return pred, err_type
    return (), Agent.VALID
  
  def export_tracks(self):
    '''
    Exports the recorded tracks from the agent's object tracker
    returns a LOCO formatted json object
    '''
    self.obj_tracker.close_all_tracks()
    self.obj_tracker.link_all_tracks(0)
    e = self.obj_tracker.export_loco_fmt()
    return e
    