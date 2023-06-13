#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *


def mouse_test(screen, environment, measurement_rate = 1):
  pt = None
  last_pt = None
  counter = 0
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        environment.serialize_agent_tracks()
        sys.exit()
      
      pt = pygame.mouse.get_pos()
      if pt == last_pt:
        continue
      last_pt = pt

      pafn.clear_frame(screen)
      pafn.frame_draw_dot(screen, pt, pafn.colors["green"], 0, 6)

      for k in environment.agents:
        sensing_agent = environment.agents[k]
        
        r,t = sensing_agent.tracker_query()
        sensing_agent.reposition(r,t)
        sensing_agent.heartbeat()
        
        render_predictions(screen, sensing_agent)

        draw_sensing_agent(screen, sensing_agent)
      pygame.display.update()
      environment.targets[0].origin = pt
      counter += 1
      if not counter % measurement_rate:
        environment.visible_targets()


def main():
  pygame.init()
  screen = pafn.create_display(1000, 1000)
  pafn.clear_frame(screen)
  sensing_agents = {}
  sa = init_sensing_agent(origin=(400,400))
  sensing_agents[sa._id] = sa
  t = init_target()
  env = init_environment(sensing_agents=sensing_agents, targets=[t])
  md = 1
  if len(sys.argv) > 1:
    md = int(sys.argv[-1])
  mouse_test(screen, env, md)

if __name__ == '__main__':
  main()