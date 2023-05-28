#!/usr/bin/python3
import numpy as np
class Filter:
  def __init__(self,
              x_arr = None,
              y_arr = None,
              z_arr = None,
              theta_arr = None,
              error_arr = None
              ):
    self.x_reg = x_arr if x_arr != None else []
    self.x_disp = x_disp if x_disp != None else []

    self.y_reg = y_arr if y_arr != None else []
    self.y_disp = y_disp if y_disp != None else []
    
    self.z_reg = z_reg if z_reg != None else []
    self.z_disp = z_disp if z_disp != None else []
    
    self.theta_reg = theta_arr if theta_arr != None else []
    self.error_reg = error_arr if error_arr != None else []
    
  def add_measurement(self, x=0, y=0, z=0, theta=0, error=0):
    self.x_reg.append(x)
    self.y_reg.append(y)
    self.z_reg.append(z)
    self.theta_reg.append(theta)
    self.error_reg.append(error)
