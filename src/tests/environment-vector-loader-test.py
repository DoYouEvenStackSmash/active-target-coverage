#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

# load points
point_lists = []
for filename in sys.argv[1:]:
  p = load_json_file(filename)
  posns = []
  for i,pt in enumerate(p):
    x,y,z,w,h = pt
    posns.append(Detection(Position(x,y,z), YoloBox(0,[x,y,w,h],i)))  
  # posns.reverse()
  point_lists.append(posns)
# convert to detections



# init window for viewing  
pygame.init()
screen = pafn.create_display(1000,1000)
zero_det = Detection(Position(-1,-1,-1), YoloBox(0,[-1,-1,-1,-1],i))
# init targets
targets = []
for posns in point_lists:
  t = Target(posns[0])
  t.path = posns
  targets.append(t)

sensing_agents = {}
sa = init_sensing_agent(origin=Position(0,0,0))
sensing_agents[sa._id] = sa
sa.heartbeat()
env = init_environment(sensing_agents=sensing_agents, targets=targets)
flag = True
counter = 0
while flag:
  flag = False
  for k in env.agents:
    sa = env.agents[k]
    # for n in range(5):
    sa.heartbeat()
    render_predictions(screen, sa)
    print(sa.get_clock())
  pygame.display.update()
  for i,t in enumerate(env.targets):
    flag = t.step() or flag
    if i == 1 and counter % 17:
      t.origin = zero_det
      t.path[t.idx] = zero_det
  
  if not counter % 3 or not counter % 7:
    env.visible_vertical_targets()
  
  print(sa.estimate_next_detection())
  counter += 1
  time.sleep(0.08)


import json
while 1:
  for event in pygame.event.get():
    if event.type == pygame.MOUSEBUTTONDOWN:
      env.serialize_agent_tracks()
      
      sys.exit()

