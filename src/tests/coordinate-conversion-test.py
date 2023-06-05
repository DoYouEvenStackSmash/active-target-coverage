#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

from drawing_functions import *

from support.file_loader import *

from StreamingAnnotations import StreamingAnnotations as sann

from YoloBox import *

def environment_coordinate_mouse_test(screen, sim_env, _id=0):
  last_pt = None
  while 1:
    for event in pygame.event.get():
      pt = pygame.mouse.get_pos()
      if pt == last_pt:
        continue
      last_pt = pt
      npt = Position(pt[0],pt[1], 0)
      viz_check(screen, sim_env, npt)
      pygame.display.update()


def viz_check(screen, sim_env, npt, pt=(0,0)):
  _id = 0
  pt = gfn.reduce_dimension(npt.get_cartesian_coordinates())
  
  if sim_env.check_grid_visibility(_id, npt):
    sa = sim_env.get_agent(_id)
    yb = sim_env.create_agent_yolobox(_id, npt)
    det = sa.create_detection(yb, True)
    result, flags = sa.is_detectable_fov(det.get_position())
    sf2 = sa.transform_from_det_coord(det)
    at2 = sa.transform_from_frame_coord(sf2)
    pt = sim_env.convert_from_agent_coordinates(0,at2)
    pt = gfn.reduce_dimension(pt.get_cartesian_coordinates())
    # at = sa.transform_
    if result:
      pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
    else:
      pafn.frame_draw_dot(screen, pt, pafn.colors["tangerine"])
  else:
    pafn.frame_draw_cross(screen, pt, pafn.colors["red"])

def loaded_coordinate_mouse_test(screen, sim_env, _id=0):
    """
    Step through all targets in the environment
    """
    flag = True
    while flag:
        flag = False
        # sim_env.visible_vertical_targets()
        for i in range(len(sim_env.targets)):
            t = sim_env.targets[i]
            # pafn.frame_draw_dot(screen, t.get_position(), pafn.colors["red"])
            viz_check(screen, sim_env, t.get_position())
            flag = t.step() or flag
        pygame.display.update()
        time.sleep(0.01)
    sys.exit()



def main():
  o = 1000
  pygame.init()
  screen = pafn.create_display(o,o)
  pafn.clear_frame(screen)
  sa = init_sensing_agent(0,origin=Position(o/2,o/2,0),max_x=40,max_y=40)
  sensing_agents = {}
  sensing_agents[sa._id] = sa
  targets = []
  
  for i, file in enumerate(sys.argv[1:]):
      p = load_json_file(file)
      posns = []
      for i, pt in enumerate(p):
        x, y, z, w, h = pt
        posns.append(Position(x,y,z))
      t = init_target(_id=i, path=posns)
      targets.append(t)
  sim_env = init_sim_environment(sensing_agents=sensing_agents, targets=targets)
  
  draw_sensing_agent(screen, sa)
  pygame.display.update()
  # environment_coordinate_mouse_test(screen,sim_env)
  loaded_coordinate_mouse_test(screen, sim_env)
  


if __name__ == '__main__':
  main()
