#!/usr/bin/python3
import numpy as np
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn

def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta

class Sensor:
  VALID = 0
  ANGULAR = 1
  RANGE = 2
  TOLERANCE = 0.1
  WINDOW_WIDTH = 100

  def __init__( self,
                sensor_origin = [0,0], 
                sensor_theta = 0,
                sensor_radius = 300,
                sensor_width = np.pi / 4):
    self.origin = sensor_origin
    self.fov_theta = sensor_theta
    self.fov_radius = sensor_radius
    self.fov_width = sensor_width
  
  def get_visible_fov(self, levels = 5):
    '''
    Gets the coordinate frame of the sensor
    Returns a list of lists of points [[(x1,y1),(x2,y2),...],[...],...]
    '''
    y_step = self.fov_radius / levels
    axes = []
    for i in range(1, levels + 1):
      axes.append(self.get_horizontal_axis(y_step * i))
    return axes

  def get_detectable_bounds(self, levels = 5):
    y_step = self.fov_radius / levels
    axes = []
    for i in range(1, levels + 1):
      axes.append(self.get_tolerance_axis(y_step * i))
    return axes

  def get_horizontal_axis(self, radius):
    '''
    Helper function for get_sensor_field
    Gets a horizontal line from the Sensor's fov
    Returns a list of points
    '''
    fov_offts = [ adjust_angle(self.fov_theta + self.fov_width / 2 ), 
                  adjust_angle(self.fov_theta + self.fov_width / 4),
                  adjust_angle(self.fov_theta),
                  adjust_angle(self.fov_theta - self.fov_width / 4),
                  adjust_angle(self.fov_theta - self.fov_width / 2 ) 
                ]
    horizontal_axis = []

    for i in fov_offts:
      horizontal_axis.append(mfn.pol2car(self.origin, radius, i))
    return horizontal_axis
  
  def get_tolerance_axis(self, radius):
    fov_offts = [ adjust_angle(self.fov_theta + self.fov_width / 2 ),
                  adjust_angle(self.fov_theta + self.fov_width / 2 - self.fov_width * Sensor.TOLERANCE), 
                  adjust_angle(self.fov_theta - self.fov_width / 2 + self.fov_width * Sensor.TOLERANCE),
                  adjust_angle(self.fov_theta - self.fov_width / 2 )
                ]
    horizontal_axis = []

    for i in fov_offts:
      horizontal_axis.append(mfn.pol2car(self.origin, radius, i))
    return horizontal_axis

  def adjust_origin(self, displacement):
    '''
    Adjusts orientation of a sensor by 
    Does not return
    '''
    self.origin = mfn.pol2car(self.origin, displacement, self.fov_theta)

  def adjust_orientation(self, theta):
    '''
    Adjusts rotation of a sensor
    Does not return
    '''
    self.fov_theta = theta
  
  def transform_to_local_coord(self, target):
    '''
    Calculates detection coordinates relative to Sensor
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
    
    x = Sensor.WINDOW_WIDTH * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]
  
  def transform_from_local_coord(self, x, y, w=1, h=1):
    '''
    Transforms a bbox from Sensor local coordinates to world coordinates
    returns a Point
    '''
    org_theta = mfn.correct_angle(self.fov_theta)
    
    rh = org_theta - self.fov_width / 2
    lh = org_theta + self.fov_width / 2
    theta = (x / Sensor.WINDOW_WIDTH) * self.fov_width
    # print(f"is_not_visible: {lh}:{theta}:{rh}")
    ratio = theta - 0.5
    theta = lh - theta
    theta = mfn.correct_angle(theta)
    
    r =  y
    return mfn.pol2car(self.origin, r, theta)

  def is_detectable(self, target):
    '''
    Indicates whether a target point is detectable (within tolerance)
    Returns a boolean indicator and a type identifier
    '''
    adj_win_bnd = Sensor.WINDOW_WIDTH * Sensor.TOLERANCE  
    adj_rad_bnd = self.fov_radius * (Sensor.TOLERANCE/1)

    if target[0] < 0 + adj_win_bnd or target[0] > Sensor.WINDOW_WIDTH - adj_win_bnd:
      return False, Sensor.ANGULAR

    if target[1] > self.fov_radius - adj_rad_bnd or target[1] < 0 + adj_rad_bnd:
      return False, Sensor.RANGE
    
    return True, Sensor.VALID
  
  def is_visible(self, target):
    '''
    Determines whether a target point is in the Sensor's sensor fov
    Returns true/false
    '''
    tar_theta, tar_r = mfn.car2pol(self.origin, target)
    
    tar_theta = adjust_angle(tar_theta)
    
    org_theta = mfn.correct_angle(self.fov_theta)

    lh = org_theta + self.fov_width / 2
    rh = org_theta - self.fov_width / 2
    if tar_theta < 0 and lh > 0 and rh > 0:
      tar_theta = 2 * np.pi + tar_theta
    if tar_r >= self.fov_radius:
      return False
    if lh > tar_theta and rh < tar_theta:
      return True
    
    return False