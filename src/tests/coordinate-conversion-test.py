#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

from drawing_functions import *

from support.file_loader import *

from StreamingAnnotations import StreamingAnnotations as sann

from YoloBox import *

def environment_coordinate_map_test(screen, sim_env, _id=0):
  last_pt = None
  while 1:
    for event in pygame.event.get():
      pt = pygame.mouse.get_pos()
      if pt == last_pt:
        continue
      last_pt = pt
      npt = Position(pt[0],pt[1], 0)
      if sim_env.check_grid_visibility(_id, npt):
        sa = sim_env.get_agent(_id)
        yb = sim_env.create_agent_yolobox(_id, npt)
        det = sa.create_detection(yb, True)
        
        result, flags = sa.is_detectable_fov(det.get_position())
        if result:
          pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
        else:
          pafn.frame_draw_dot(screen, pt, pafn.colors["tangerine"])
      else:
        pafn.frame_draw_cross(screen, pt, pafn.colors["red"])
      pygame.display.update()

def main():
  o = 1000
  pygame.init()
  screen = pafn.create_display(o,o)
  pafn.clear_frame(screen)
  sa = init_sensing_agent(0,origin=Position(o/2,o/2,0),max_x=100,max_y=100)
  sensing_agents = {}
  sensing_agents[sa._id] = sa

  sim_env = init_sim_environment(sensing_agents=sensing_agents)
  # print(sa.get_center().get_cartesian_coordinates())
  draw_sensing_agent(screen, sa)
  pygame.display.update()
  environment_coordinate_map_test(screen,sim_env)
  # agent_coordinate_map_test(screen,sa)


if __name__ == '__main__':
  main()
