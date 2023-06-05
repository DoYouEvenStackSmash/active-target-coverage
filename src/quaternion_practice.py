#!/usr/bin/python3
import numpy as np
class Quaternion:
  def __init__ (self,w=0,x=0,y=0,z=0):
    """
    Initializer for quaternion
    """
    self.w = w
    self.x = x
    self.y = y
    self.z = z

  def multiply(self, q):
    """
    Multiply quaternions
    """
    w1,x1,y1,z1 = self.get_components()
    w2,x2,y2,z2 = q.get_components()
    
    w3 =w1*w2 - x1*x2 - y1*y2 - z1*z2 
    x3 =w1*x2 +w2*x1 + y1*z2 - y2*z1
    y3 =w1*y2 +w2*y1 + x2*z1 - x1*z2
    z3 =w1*z2 +w2*z1 + x1*y2 - x2*y1
    
    return Quaternion(w3,x3,y3,z3)
  
  def get_components(self):
    """
    Get quaternion components
    """
    return [self.w,self.x,self.y,self.z]
  
  def get_angle_axis(self):
    """
    Get anglewxis representation of quaternion
    """
    w,x,y,z = self.get_components()
    denom = np.sqrt(np.square(x) + np.square(y) + np.square(z))
    x = np.divide(x,denom)
    y = np.divide(y,denom)
    z = np.divide(z,denom)
    theta = 2 * np.arctan2(denom,w)
    return theta,x,y,z

  def to_json(self):
    w,x,y,z = self.get_components()

    return {"w":w,"x":x,"y":y,"z":z}

def q_test():
  q0 = Quaternion(0,0,0,1)
  print(f"0: {q0.get_angle_axis()}")
  # q2 = Quaternion(np.pi/2,1,1,0)
  q1 = q0.multiply(q0)
  print(f"1: {q1.get_angle_axis()}")
  q2 = q1.multiply(q0)
  print(f"2: {q2.get_angle_axis()}")
  q3 = q2.multiply(q0)
  print(f"3: {q3.get_angle_axis()}")
  q4 = q3.multiply(q0)
  print(f"4: {q4.get_angle_axis()}")
  print("\nrotating again")
  q5 = q4.multiply(q0)
  print(f"5: {q5.get_angle_axis()}")
  q6 = q5.multiply(q0)
  print(f"6: {q6.get_angle_axis()}")
  q7 = q6.multiply(q0)
  print(f"7: {q7.get_angle_axis()}")
  q8 = q7.multiply(q0)
  print(f"8: {q8.get_angle_axis()}")
  print(f"0: {q0.get_angle_axis()}")

def main():
  q_test()

if __name__ == '__main__':
  main()