#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

# def environment_agent_update(environment, FORCE_UPDATE=False):
#   for k in environment.agents:
#       sensing_agent = environment.agents[k]
#       if sensing_agent.ALLOW_PREDICTION == FORCE_UPDATE:
#         r,t = sensing_agent.tracker_query()
#         sensing_agent.reposition(r,t)
#         sensing_agent.heartbeat()
  
# def environment_agent_illustration(screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts):
#   for k in environment.agents:
#     sensing_agent = environment.agents[k]
#     render_predictions(screen, sensing_agent)
#     # demo rendering
#     accumulate_predictions(sensing_agent, curr_pts, pred_pts)
#     for i in range(max(0, len(marked_pts) - 5), len(marked_pts)):
#       pafn.frame_draw_dot(screen, marked_pts[i], pafn.colors["tangerine"], 0, (1 - (len(marked_pts) - i) / 5) * 10)
#     # if len(curr_pts):
#     #   pafn.frame_draw_cross(screen, curr_pts[-1], pafn.colors["tangerine"], 20)
#     for idx in range(max(0, len(pred_pts) - int(measurement_rate * DELAY)), len(pred_pts), 3):
#       pafn.frame_draw_dot(screen, pred_pts[idx], pafn.colors["yellow"], 0, 4)
    
#     draw_sensing_agent(screen, sensing_agent)

DELAY=1
def static_mouse_test(screen, environment, measurement_rate = 1):
  pt = None
  last_pt = (0,0)#None
  pred_pts = []
  curr_pts = []
  marked_pts = []
  counter = 0
  
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        environment.serialize_agent_tracks()
        sys.exit()
      
      pt = pygame.mouse.get_pos()
    
      pafn.clear_frame(screen)
      
    
      environment_agent_update(environment, True)
      # environment_agent_illustration(screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts)
      
      environment.targets[0].origin = pt
      counter += 1
      if not counter % measurement_rate:
        last_pt = pt
        if len(curr_pts):
          marked_pts.append(curr_pts[-1])
        # if not PREDICT:
        environment_agent_update(environment)
        environment.visible_targets()
      environment_agent_illustration(screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts)
      pafn.frame_draw_dot(screen, last_pt, pafn.colors["green"], 4, 8)
      pygame.display.update()

def main():
  pygame.init()
  screen = pafn.create_display(1000, 1000)
  pafn.clear_frame(screen)
  sensing_agents = {}
  sa = init_sensing_agent(_id=1,origin=(480,900),width=np.pi,radius=7000)
  sa.rotate_agent((500,500))
  sa2 = init_sensing_agent(_id=2,origin=(480,900), width=np.pi/3, radius=300)
  sa2.rotate_agent((500,500))
  sa3 = init_sensing_agent(_id=3,origin=(50,700),radius=150)
  sa.ALLOW_ROTATION=False
  # sa2.ALLOW_TRANSLATION=False
  # sa2.ALLOW_PREDICTION=False
  sa._id = 1
  sa2._id = 2
  sa3._id = 3
  sa2.heartbeat()
  # sa2.exoskeleton.color = pafn.colors["pink"]
  sa2.exoskeleton.color = pafn.colors["yellow"]
  # sa2.exoskeleton.color = (255,255,255)
  # sensing_agents[sa._id] = sa
  sensing_agents[sa2._id] = sa2
  # sensing_agents[sa3._id] = sa3

  t = init_target()
  env = init_environment(sensing_agents=sensing_agents, targets=[t])
  md = 1
  if len(sys.argv) > 1:
    md = int(sys.argv[-1])
  static_mouse_test(screen, env, md)

if __name__ == '__main__':
  main()