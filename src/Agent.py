#!/usr/bin/python3


from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
import numpy as np
  
def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta

class Agent:
  def __init__(self, origin = [0, 0], fov = [0, 100, np.pi / 2], _id = 0):
    self.origin = origin
    self.fov_theta = fov[0]
    self.fov_radius = fov[1]
    self.fov_width = fov[2]
    self._id = _id
  

  def rotate(self, target):
    self.fov_theta = mfn.car2pol(self.origin, target)[0]

  def get_polygon(self):  
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

  def is_visible(self, target):
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