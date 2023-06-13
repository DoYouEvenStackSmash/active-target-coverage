#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *


def active_mouse_test(screen, environment, measurement_rate = 1):
  pt = None
  last_pt = None
  counter = 0
  curr_pts = []
  pred_pts = []
  marked_pts = []
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        environment.serialize_agent_tracks()
        sys.exit()
      
      pt = pygame.mouse.get_pos()
      # if pt == last_pt:
      #   # counter+=1
      #   continue
      last_pt = pt

      pafn.clear_frame(screen)
      environment_agent_update(environment, True)
      # pafn.frame_draw_dot(screen, pt, pafn.colors["green"], 0, 8)

      # for k in environment.agents:
      #   sensing_agent = environment.agents[k]
        
      #   r,t = sensing_agent.tracker_query()
      #   sensing_agent.reposition(r,t)
      #   sensing_agent.heartbeat()
        
      #   render_predictions(screen, sensing_agent)

      #   draw_sensing_agent(screen, sensing_agent)
      
      environment.targets[0].origin = pt
      counter += 1
      if not counter % measurement_rate:
        marked_pts.append(pt)
        environment.visible_targets()
      environment_agent_illustration(screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts)
    pygame.display.update()

def main():
  pygame.init()
  screen = pafn.create_display(1400, 1000)
  pafn.clear_frame(screen)
  sensing_agents = {}
  sa = init_sensing_agent(origin=(0,400))
  sensing_agents[sa._id] = sa
  t = init_target()
  env = init_environment(sensing_agents=sensing_agents, targets=[t])
  md = 1
  if len(sys.argv) > 1:
    md = int(sys.argv[-1])
  active_mouse_test(screen, env, md)

if __name__ == '__main__':
  main()