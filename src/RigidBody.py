#!/usr/bin/python3
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *

class RigidBody:
  LINE_LEN = 30
  def __init__(self, 
              ref_origin = (0,0),
              endpoint = (0,0),
              ref_center = (0,0),
              point_set = [],
              rigid_body = None,
              _prev = None,
              _next = None, 
              theta = 0
              ):
    self.origin = ref_origin
    self.ref_endpoint = endpoint
    self.ref_center = ref_center
    self.point_set = point_set
    self.body = rigid_body
    self.rel_theta = theta
    self.prev = _prev
    self.next = _next
    self.endpoint = endpoint
  
  def get_body(self):
    '''
    Accessor for internal polygon
    Returns a polygon object or None
    '''
    return self.body

  def get_points(self):
    '''
    Wrapper for points access
    '''
    if self.body == None:
      return self.get_point_set()
    return self.get_body_points()
  
  def get_point_set(self):
    '''
    Accessor for the point set which is constrained to the RigidBody
    Returns a list of points
    '''
    return self.point_set
  
  def get_body_points(self):
    '''
    Get internal point set
    returns a list of points
    '''
    return self.body.dump_points()
  
  def get_relative_angle(self):
    '''
    Accessor for the RigidBody's angle relative to previous RigidBody
    Returns an angle theta
    '''
    return self.rel_theta
  
  def get_rotation_center(self):
    '''
    Accessor for the RigidBody's origin
    returns a point
    '''
    return self.ref_center

  def get_endpoint(self):
    '''
    Accessor for the RigidBody endpoint
    Returns a point
    '''
    return self.ref_endpoint
  
  def get_origin(self):
    return self.origin

  def rotate_on_center(self, displacement):
    rot_mat = tfn.calculate_rotation_matrix(displacement)
    # rad,pt = tfn.calculate_rotation(R.get_rotation_center(), o ,last)
  
  
  
    self.rotate_body(self.get_rotation_center(), rot_mat)
    tfn.rotate_point(self.get_rotation_center(), R.get_origin(), rot_mat)
    tfn.rotate_point(self.get_rotation_center(), R.get_endpoint(), rot_mat)

  def rotate(self, origin, rot_mat):
    '''
    Given a rotation matrix, rotate the RigidBody about the given origin
    Does not return
    '''
    self.endpoint = tfn.rotate_point(origin, self.get_endpoint(), rot_mat)
    self.point_set = tfn.rotate_point_set(origin, self.point_set, rot_mat)

  
  def rotate_body(self, origin=(0,0), theta = 0, rot_mat = None):
    '''
    Rotates internal polygon
    Does not return
    '''
    
    if rot_mat == None and theta != 0:
      rot_mat = tfn.calculate_rotation_matrix(theta,1)
      self.rel_theta = self.rel_theta + theta
    
    rotate_polygon(self.body, rot_mat, origin)
    self.origin = tfn.rotate_point(origin, self.get_origin(), rot_mat)
    self.ref_endpoint = tfn.rotate_point(origin, self.get_endpoint(), rot_mat)
    self.ref_center = tfn.rotate_point(origin, self.get_rotation_center(), rot_mat)
    return theta
    
    
    
  
  def translate_body(self, x_disp, y_disp):
    '''
    Translates internal polygon
    Does not return
    '''
    self.endpoint = (self.endpoint[0] + x_disp, self.endpoint[1] + y_disp)
    translate_polygon(self.body, x_disp, y_disp)
    

  def update_orientation(self, point_set, theta):
    '''
    TODO rotate by angle
    '''
    self.point_set = point_set
    self.rel_theta = theta
  
  def update_point_set(self, point_set):
    '''
    Replace existing point set with a new point set.
    Used during rotations
    Does not return
    '''
    self.point_set = point_set

  def get_normals(self):
    '''
    Calculates coordinate axes x,y in R2
    Returns a pair of points representing axis unit endpoints
    '''
    ox,oy = self.get_rotation_center()
    theta = self.rel_theta
    xx,xy = RigidBody.LINE_LEN * np.cos(theta), RigidBody.LINE_LEN * np.sin(theta)
    yx,yy = RigidBody.LINE_LEN * np.cos(theta + np.pi / 2), RigidBody.LINE_LEN * np.sin(theta + np.pi / 2)
    return ((xx+ox,xy+oy), (yx+ox,yy+oy))

  def get_relative_rotation(self, target_point):
    '''
    Given a target point, computes the angle theta between the current
    endpoint and the target point for use during rotation

    Returns a normalized angle theta
    '''
  
    norm, dist = MathFxns.car2pol(self.get_origin(), self.get_endpoint())
    rad, r = MathFxns.car2pol(self.get_origin(), target_point)
    
    norm = MathFxns.correct_angle(norm)
    rad = MathFxns.correct_angle(rad)
    
    rotation = np.subtract(rad,norm)
    
    # correction for arctan identification
    if rotation > np.pi:
      rotation = rotation - 2 * np.pi
    if rotation < -np.pi:
      rotation = rotation + 2 * np.pi
    
    return rotation

    