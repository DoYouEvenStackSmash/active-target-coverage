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
import pygame
import time

def draw_all_normals(screen, R):
  '''
  Render function for all coordinate frames in a chain
  Does not return
  '''
  n = R.get_normals()
  pafn.frame_draw_line(screen, (R.get_rotation_center(), n[0]), pafn.colors['tangerine'])
  pafn.frame_draw_line(screen, (R.get_rotation_center(), n[1]), pafn.colors['yellow'])
  


def draw_all_links(screen, link):
  '''
  Render function for all polygon links in the chain
  Does not return
  '''
  points = link.get_points()
  
  pafn.frame_draw_polygon(screen, points, pafn.colors["red"])


def repeatable_link_test(screen, R):
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  # origins = [(450, 500), (500, 450), (550, 500), (500,550)]
  last = (550,500)
  for o in target_points:
    pafn.clear_frame(screen)
    pafn.frame_draw_dot(screen, o, pafn.colors["green"])
    pygame.display.update()
    if last != None:
      rad,pt = tfn.calculate_rotation(R.get_rotation_center(), o ,last)
    else:
      rad = R.get_relative_rotation(o)
    last = o
    print(R.rotate_body(R.get_rotation_center(), theta=rad))
    # print(rad)
    # rot_mat = tfn.calculate_rotation_matrix(rad)
    
    # pafn.clear_frame(screen)
    pafn.frame_draw_dot(screen, o, pafn.colors["green"])

      # R.rotate_body(R.get_rotation_center(), rot_mat)
      # tfn.rotate_point(R.get_rotation_center(), R.get_origin(), rot_mat)
      # tfn.rotate_point(R.get_rotation_center(), R.get_endpoint(), rot_mat)
    
    draw_all_normals(screen, R)
    draw_all_links(screen, R)
    
    pygame.display.update()  
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
  a = RigidBody(ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_body = ap)
  a.prev = a
  repeatable_link_test(screen, a)
  

if __name__ == '__main__':
  main()