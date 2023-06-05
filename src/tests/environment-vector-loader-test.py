#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

from drawing_functions import *

from support.file_loader import *

# load points
point_lists = []
for filename in sys.argv[1:]:
    p = load_json_file(filename)
    posns = []
    for i, pt in enumerate(p):
        x, y, z, w, h = pt
        posns.append(Detection(Position(x, y, z), YoloBox(0, [x, y, w, h], i)))
    # posns.reverse()
    point_lists.append(posns)
# convert to detections


# init window for viewing
pygame.init()
screen = pafn.create_display(1000, 1000)
zero_det = Detection(Position(-1, -1, -1), YoloBox(0, [-1, -1, -1, -1], i))
# init targets
targets = []
for posns in point_lists:
    t = Target(posns[0])
    t.path = posns
    targets.append(t)

sensing_agents = {}
sa = init_sensing_agent(origin=Position(0, 0, 0), max_x=100, max_y=100)
sensing_agents[sa._id] = sa
sa.heartbeat()
sim_env = init_sim_environment(sensing_agents=sensing_agents, targets=targets)
flag = True
counter = 0

while flag:
    flag = False
    for k in sim_env.agents:

        sa = sim_env.agents[k]
        # for n in range(5):
        sa.heartbeat()
        arr = []
        arr = sa.estimate_next_detection()
        if not len(arr):
          continue
        for i in range(len(arr)):
          curr_pt, pred_pt = arr[i]
          cppt = sa.transform_from_frame_coord(curr_pt)
          ppt = sa.transform_from_frame_coord(pred_pt)
          predpt = sim_env.convert_from_agent_coordinates(0, ppt)
          currpt = sim_env.convert_from_agent_coordinates(0, cppt)
          crpt = gfn.reduce_dimension(currpt.get_cartesian_coordinates())
          prpt = gfn.reduce_dimension(predpt.get_cartesian_coordinates())
          pafn.frame_draw_dot(screen, crpt, pafn.colors["tangerine"])
          pafn.frame_draw_dot(screen, prpt, pafn.colors["yellow"])
          pafn.frame_draw_line(screen, [crpt,prpt], pafn.colors["white"])

        
        print(sa.get_clock())
    pygame.display.update()
    for i, t in enumerate(sim_env.targets):
        flag = t.step() or flag
        if i == 1 and counter % 17:
            t.origin = zero_det
            t.path[t.idx] = zero_det

    if not counter % 7:
      sim_env.visible_vertical_targets()

    # print(sa.estimate_next_detection())

    counter += 1
    time.sleep(0.08)


import json

while 1:
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            sim_env.serialize_agent_tracks()

            sys.exit()
