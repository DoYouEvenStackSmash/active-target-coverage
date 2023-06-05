#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

GLOBAL_ORIGIN = (0,0,0)
def convert_to_sensor_coord(sa, pt):
  
  pt = Position(pt[0],pt[1], 0)
  theta, radius = mfn.car2pol(sa.get_origin().get_cartesian_coordinates(), pt.get_cartesian_coordinates())
  # print(theta)

  x1,y1,z1 = mfn.pol2car(GLOBAL_ORIGIN, radius, theta)
  new_pt = Position(x1,y1,z1)
  return new_pt

def convert_to_angle_offset(posn):
  theta, radius = mfn.car2pol(GLOBAL_ORIGIN, posn.get_cartesian_coordinates())
  return (theta, 0, radius)

def convert_to_sensor_frame_coords(sa, posn):
  theta, phi, rad = convert_to_angle_offset(posn)
  x = rad
  y = theta / sa.get_fov_width() * sa.get_max_x() + sa.get_max_x() / 2
  z = phi / sa.get_fov_height() * sa.get_max_y() + sa.get_max_y() / 2
  new_posn = Position(x,y,z)
  return new_posn

def coordinate_map_test(screen, sa):
  last_pt = None
  while 1:
    for event in pygame.event.get():
      pt = pygame.mouse.get_pos()
      if pt == last_pt:
        continue
      last_pt = pt
      # pafn.frame_draw_dot(screen, pt, pafn.colors["green"])

      npt = convert_to_sensor_coord(sa,pt)
      theta, phi, rad = convert_to_angle_offset(npt)
      # print(sa.get_fov_width())
      # print(theta)
      sensor_posn = convert_to_sensor_frame_coords(sa,npt)

      print(sensor_posn.get_cartesian_coordinates())

      # if sa.is_visible_fov(theta, phi, rad):
      #   pafn.frame_draw_dot(screen, pt, pafn.colors["cyan"],4,8)
      # else:
      #   pafn.frame_draw_dot(screen, pt, pafn.colors["red"],4,8)
      
      result, flags = sa.is_detectable_fov(sensor_posn)
      if result:
        pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
      else:
        pafn.frame_draw_dot(screen, pt, pafn.colors["tangerine"])
      
      # print(npt.get_cartesian_coordinates())
      pygame.display.update()


def main():
  o = 1000
  pygame.init()
  screen = pafn.create_display(o,o)
  pafn.clear_frame(screen)
  sa = init_sensing_agent(0,origin=Position(o/2,o/2,0),max_x=100,max_y=100)
  print(sa.get_center().get_cartesian_coordinates())
  draw_sensing_agent(screen, sa)
  pygame.display.update()
  coordinate_map_test(screen,sa)

  time.sleep(5)

if __name__ == '__main__':
  main()
