#!/usr/bin/python3
class Detection:
  """ An abstraction for describing a detection
    
  """
  def __init__( self, 
                position,
                attributes):
    self.position = position
    self.attributes = attributes
    self.parent_track = None
    self._id = None

  def get_attributes(self):
    return self.attributes

  def get_position(self):
    return self.position
  
  def get_cartesian_coord(self):
    return self.position.get_center_coord()
  
  def get_attr_coord(self):
    return self.attributes.get_center_coord()
  
  def set_attributes(self, attr):
    self.attributes = attr
  
class Position:
  """
  An abstraction for describing a position
  """
  def __init__(self, x=0, y=0, z=0, theta=0, phi=0):
    self.x = x
    self.y = y
    self.z = z
    self.theta = theta
    self.phi = phi

  def get_center_coord(self):
    return (self.x, self.y)

  
