#!/usr/bin/python3
import numpy as np
class Quaternion:
  def __init__ (self, a=0,b=0,c=0,d=0):
    """
    Initializer for quaternion
    """
    self.a = a
    self.b = b
    self.c = c
    self.d = d

  def multiply(self, q):
    """
    Multiply quaternions
    """
    a1,b1,c1,d1 = self.get_components()
    a2,b2,c2,d2 = q.get_components()
    
    a3 = a1*a2 - b1*b2 - c1*c2 - d1*d2 
    b3 = a1*b2 + a2*b1 + c1*d2 - c2*d1
    c3 = a1*c2 + a2*c1 + b2*d1 - b1*d2
    d3 = a1*d2 + a2*d1 + b1*c2 - b2*c1
    
    return Quaternion(a3,b3,c3,d3)
  
  def get_components(self):
    """
    Get quaternion components
    """
    return [self.a,self.b,self.c,self.d]
  
  def get_angle_axis(self):
    """
    Get angle axis representation of quaternion
    """
    a,b,c,d = self.get_components()
    denom = np.sqrt(np.square(b) + np.square(c) + np.square(d))
    b = b / denom
    c = c / denom
    d = d / denom
    theta = 2 * np.arctan2(denom, a)
    return b,c,d,theta

def q_test():
  q1 = Quaternion(1,0,0,0.7)
  q2 = Quaternion(1,0,0,0.7)
  q3 = q2.multiply(q1)
  q4 = Quaternion(1,0,0,-4.7)
  q3 = q3.multiply(q4)

  b,c,d,theta = q3.get_angle_axis()
  print(b)
  print(c)
  print(d)
  print(theta)


  
