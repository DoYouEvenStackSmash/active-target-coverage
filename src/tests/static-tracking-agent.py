#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

def accumulate_predictions(sensing_agent, curr_pts, pred_pts):
  curr_pt, pred_pt = (),()
  arr = sensing_agent.estimate_next_detection()
  if len(arr):
      curr_pt = arr[0][0]
      pred_pt = arr[0][1]
  else:
    return
  if len(pred_pt):
    curr_pts.append(curr_pt)
    pred_pts.append(pred_pt)
  else:
    curr_pts.append(curr_pt)
    pred_pts.append(curr_pt)

def tracking_test(screen, environment, interval=0):
  flag = True
  counter = 0
  curr_pts = []
  pred_pts = []
  while flag:
    flag = False
    pafn.clear_frame(screen)

    for k in environment.agents:
      sensing_agent = environment.agents[k]
      
      
      
      render_predictions(screen, sensing_agent)
      for i in sensing_agent.get_last_detections():
        pafn.frame_draw_dot(screen, i, pafn.colors["cyan"],0,8)
      accumulate_predictions(sensing_agent, curr_pts, pred_pts)
      sensing_agent.heartbeat()
      for idx in range(len(curr_pts)):
        pafn.frame_draw_dot(screen, curr_pts[idx], pafn.colors["tangerine"])
        pafn.frame_draw_dot(screen, pred_pts[idx], pafn.colors["yellow"])
        pafn.frame_draw_line(
            screen, (curr_pts[idx], pred_pts[idx]), pafn.colors["white"]
        )

      draw_sensing_agent(screen, sensing_agent)
  
    if not counter % interval:# or not counter % 9:
      for i in range(len(environment.targets)):
        t = environment.targets[i]
        pafn.frame_draw_dot(screen, t.get_position(), t.color)
      environment.visible_targets()
    for i in range(len(environment.targets)):
        t = environment.targets[i]
        # pafn.frame_draw_dot(screen, t.get_position(), t.color)
        flag = t.step() or flag
    
    pygame.display.update()
    time.sleep(0.1)
    counter += 1
  # for i in environment.agents:
  #   sensing_agent = environment.agents[i]
  #   import_agent_record(screen, sensing_agent.export_tracks())
  # pygame.display.update()

  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN: 
        environment.serialize_agent_tracks()
        sys.exit()


def main():
  pygame.init()
  screen = pafn.create_display(1000, 1000)
  pafn.clear_frame(screen)
  sensing_agents = {}
  sa = init_sensing_agent(_id=0, origin=(500,0), width=1.5 * np.pi, radius=2000)
  sa2 = init_sensing_agent(_id=1, origin=(700,150), width=1.5 * np.pi, radius=200)
  sa.rotate_agent((500,500))
  sa2.rotate_agent((500,500))
  sa.ALLOW_TRANSLATION=False
  sa.ALLOW_ROTATION=False
  sa.heartbeat()
  sa2._id = 1
  # sensing_agents[sa2._id] = sa2
  sensing_agents[sa._id] = sa
  
  targets = []
  # load json point files
  for i,file in enumerate(sys.argv[1:-1]):
    p = load_json_file(file)
    
    t = init_target(_id=i, path=p)
    targets.append(t)
  env = init_environment(sensing_agents=sensing_agents, targets=targets)
  time.sleep(0.2)
  tracking_test(screen, env, int(sys.argv[-1]))

if __name__ == '__main__':
  main()