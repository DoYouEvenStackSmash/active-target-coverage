#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link



from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
import pygame
import time

def draw_coordinate_frame(screen, S):
  '''
  Helper function for displaying the curved coordinate fov of agent A
  Does not return
  '''
  levels = 5
  coord_frame = S.get_visible_fov(levels)
  detect_frame = S.get_detectable_bounds(levels)
  for c in range(levels):
    for i in range(1,len(coord_frame[c])):
      pafn.frame_draw_line(screen, (coord_frame[c][i-1], coord_frame[c][i]), pafn.colors['white'])
      
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors['white'])
  
  for endpoint in detect_frame[-1]:
    pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors['tangerine'])


def draw_all_normals(screen, R):
  '''
  Render function for all coordinate frames in a chain
  Does not return
  '''
  n = R.get_normals()
  pafn.frame_draw_line(screen, (R.get_rotation_center(), n[0]), pafn.colors['tangerine'])
  pafn.frame_draw_line(screen, (R.get_rotation_center(), n[1]), pafn.colors['yellow'])
  pafn.frame_draw_dot(screen, R.get_endpoint(), pafn.colors['white'])


def draw_all_links(screen, link):
  '''
  Render function for all polygon links in the chain
  Does not return
  '''
  points = link.get_points()
  
  pafn.frame_draw_polygon(screen, points, pafn.colors["red"])

def draw_rigid_body(screen, R):
  draw_all_normals(screen, R)
  draw_all_links(screen, R)

def draw_sensing_agent(screen, SA):
  ex,s = SA.get_agent()
  draw_coordinate_frame(screen, s)
  draw_rigid_body(screen,ex)

def repeatable_sensing_agent_test(screen, SA):
  draw_sensing_agent(screen, SA)
  pygame.display.update()
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  offset = 50
  origin = (500,500)
  scaling = 2
  x,y = origin
  target_points = [(x - offset * scaling,y - offset * scaling), (x + offset * scaling, y - offset * scaling), (x + offset * scaling,y + offset * scaling), (x - offset * scaling, y + offset * scaling)]
  origins = [(450, 500), (500, 450), (550, 500), (500,550)]
  last = (600,500)
  draw_sensing_agent(screen, SA)
  
  for tpt in target_points:
    pafn.frame_draw_dot(screen, tpt, pafn.colors["red"])
  pygame.display.update()
  time.sleep(1)
  for o in target_points:
    pafn.clear_frame(screen)
    pafn.frame_draw_dot(screen, o, pafn.colors["green"])

    if last != None:
      rad,pt = tfn.calculate_rotation(SA.get_center(), o ,last)
    else:
      rad = R.get_relative_rotation(o)
    last = o
    SA.rotate_agent(rad)
    for tpt in target_points:
      status = SA.sensor.is_visible(tpt)
        
      if status:
        status2 = SA.sensor.is_detectable(SA.sensor.transform_to_local_coord(tpt))
        if status2:
          pafn.frame_draw_dot(screen, tpt, pafn.colors["cyan"])
          print(SA.sensor.transform_to_local_coord(tpt))
      else:
        pafn.frame_draw_dot(screen, tpt, pafn.colors["red"])
    draw_sensing_agent(screen, SA)
    pygame.display.update()
    
    
    # print(rad)
    # rot_mat = tfn.calculate_rotation_matrix(rad)
    
    # pafn.clear_frame(screen)
    pafn.frame_draw_dot(screen, o, pafn.colors["green"])

      # R.rotate_body(R.get_rotation_center(), rot_mat)
      # tfn.rotate_point(R.get_rotation_center(), R.get_origin(), rot_mat)
      # tfn.rotate_point(R.get_rotation_center(), R.get_endpoint(), rot_mat)
    

    
    # pygame.display.update()  
    time.sleep(0.01)
    
    time.sleep(1)




def main():
  pygame.init()
    
  screen = pafn.create_display(1000,1000)
  origin = (500,500)
  ox,oy = origin
  scale = 3
  opts = [(ox - 10*scale, oy - 10*scale), (ox - 10*scale, oy + 10*scale), (ox + 30 * scale, oy)]
  mpt = gfn.get_midpoint(opts[0], opts[1])
  mpt2 = gfn.get_midpoint(mpt,opts[2])
  ap = Polygon(opts)
  rb = RigidBody(ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_body = ap)
  rb.prev = rb
  s = Sensor()
  s.fov_width = 3 * np.pi / 5
  s.origin = opts[2]
  A = SensingAgent(rb, s)
  
  repeatable_sensing_agent_test(screen, A)
  

if __name__ == '__main__':
  main()