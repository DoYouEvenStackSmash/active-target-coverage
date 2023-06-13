#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *



def tracking_test(screen, environment, interval=0):
  flag = True
  counter = 0
  curr_pts = []
  pred_pts = []
  marked_pts = []
  while flag:
    flag = False
    pafn.clear_frame(screen)
    environment_agent_update(environment, True)
    
    # for k in environment.agents:
    #   sensing_agent = environment.agents[k]
      
    #   # render_predictions(screen, sensing_agent)
    #   # for i in sensing_agent.get_last_detections():
    #   #   pafn.frame_draw_dot(screen, i, pafn.colors["cyan"],0,8)
    #   # accumulate_predictions(sensing_agent, curr_pts, pred_pts)
    #   # sensing_agent.heartbeat()
    #   # for idx in range(len(curr_pts)):
    #   #   pafn.frame_draw_dot(screen, curr_pts[idx], pafn.colors["tangerine"])
    #   #   pafn.frame_draw_dot(screen, pred_pts[idx], pafn.colors["yellow"])
    #   #   pafn.frame_draw_line(
    #   #       screen, (curr_pts[idx], pred_pts[idx]), pafn.colors["white"]
    #   #   )

    #   draw_sensing_agent(screen, sensing_agent)
  
    if not counter % interval:# or not counter % 9:
      for i in range(len(environment.targets)):
        t = environment.targets[i]
        pafn.frame_draw_dot(screen, t.get_position(), t.color)
        marked_pts.append(t.get_position())
      environment_agent_update(environment)
      environment.visible_targets()
    environment_agent_illustration(screen, environment, interval, curr_pts,pred_pts, marked_pts)
    for i in range(len(environment.targets)):
        t = environment.targets[i]
        # pafn.frame_draw_dot(screen, t.get_position(), t.color)
        flag = t.step() or flag
    
    pygame.display.update()
    time.sleep(0.01)
    counter += 1
  for i in environment.agents:
    sensing_agent = environment.agents[i]
    import_agent_record(screen, sensing_agent.export_tracks())
  pygame.display.update()

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
  sa = init_sensing_agent(_id=0, origin=(650,250), width=np.pi, radius=100)
  sa2 = init_sensing_agent(_id=1, origin=(650,250), width=np.pi, radius=100)
  sa.rotate_agent((500,500))
  sa2.rotate_agent((500,500))
  # sa.ALLOW_TRANSLATION=False
  # sa.ALLOW_ROTATION=False
  sa.heartbeat()
  sa2.heartbeat()
  sa.ALLOW_PREDICTION = False
  sa2.ALLOW_PREDICTION = True
  sa._id = 0
  sa2._id = 1
  sensing_agents[sa._id] = sa
  sensing_agents[sa2._id] = sa2
  
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