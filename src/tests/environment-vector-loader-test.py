#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

p = load_json_file(sys.argv[1])

posns = []
for i,pt in enumerate(p):
  x,y,z,w,h = pt
  posns.append(Detection(Position(x,y,z), YoloBox(0,[x,y,w,h],i)))
pygame.init()
screen = pafn.create_display(1000,1000)

t = Target(posns[0])
t.path = posns
sensing_agents = {}
sa = init_sensing_agent(origin=Position(500,500,0))
sensing_agents[sa._id] = sa
# t = init_target()
# print(t.get_origin())
env = init_environment(sensing_agents=sensing_agents, targets=[t])
sa.heartbeat()
for i in range(len(t.path)):
  
  sa.heartbeat()
  render_predictions(screen, sa)
  pygame.display.update()
  t.step()
  if not i % 3:
    env.visible_vertical_targets()
  
  print(sa.estimate_next_detection())
  
  time.sleep(0.08)


import json
while 1:
  for event in pygame.event.get():
    if event.type == pygame.MOUSEBUTTONDOWN:
      env.serialize_agent_tracks()
      
      sys.exit()

