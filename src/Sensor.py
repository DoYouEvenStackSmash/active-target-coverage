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
  BOTH = 3
  TOLERANCE = 0.1
  WINDOW_WIDTH = 100

  def __init__( self,
                parent_agent,
                sensor_radius = 300,
                sensor_width = np.pi / 4):
    self.parent_agent = parent_agent
    self.fov_radius = sensor_radius
    self.fov_width = sensor_width
    
  def get_origin(self):
    '''
    Accessor for parent origin
    returns a point
    '''
    origin = self.parent_agent.get_origin()
    return origin
  
  def get_fov_theta(self):
    '''
    Accessor for parent fov_theta, which is a vector
    returns a (theta, radius) tuple
    '''
    fov_theta = self.parent_agent.get_fov_theta()
    return fov_theta

  def get_fov_width(self):
    '''
    Accessor for fov_width of the sensor
    '''
    return self.fov_width

  def get_fov_radius(self):
    '''
    Accessor for scalar range
    returns a number
    '''
    return self.fov_radius
  
  def get_width(self):
    '''
    Accessor for scalar width
    returns a number
    '''
    return self.fov_width
  
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
    '''
    Gets the detectable bounds of the sensor
    Returns a list of lists of points
    '''
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
    fov_offts = [ adjust_angle(self.get_fov_theta() + self.fov_width / 2 ), 
                  adjust_angle(self.get_fov_theta() + self.fov_width / 4),
                  adjust_angle(self.get_fov_theta()),
                  adjust_angle(self.get_fov_theta() - self.fov_width / 4),
                  adjust_angle(self.get_fov_theta() - self.fov_width / 2 ) 
                ]
    horizontal_axis = []

    for i in fov_offts:
      horizontal_axis.append(mfn.pol2car(self.get_origin(), radius, i))
    return horizontal_axis
  
  def get_tolerance_axis(self, radius):
    '''
    Helper function for get_detectable_bounds
    Gets a horizontal line from the sensor's fov
    '''
    fov_offts = [ adjust_angle(self.get_fov_theta() + self.fov_width / 2 ),
                  adjust_angle(self.get_fov_theta() + self.fov_width / 2 - self.fov_width * Sensor.TOLERANCE), 
                  adjust_angle(self.get_fov_theta() - self.fov_width / 2 + self.fov_width * Sensor.TOLERANCE),
                  adjust_angle(self.get_fov_theta() - self.fov_width / 2 )
                ]
    horizontal_axis = []

    for i in fov_offts:
      horizontal_axis.append(mfn.pol2car(self.get_origin(), radius, i))
    return horizontal_axis

