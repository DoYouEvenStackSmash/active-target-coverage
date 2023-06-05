#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

target_path = [Position(1, 1, 0), Position(2, 2, 0)]

t = init_target(path=target_path)
# steps a target along its path
print(t.get_position())
t.step()

print(t.get_position())
t.step()

print(t.get_position())


sa = init_sensing_agent(_id=0, origin=Position(500, 500, 0), max_x=300, max_y=300)
sim_env = init_sim_environment(sensing_agents={0:sa})

print(sa.get_origin().get_cartesian_coordinates())
target = Position(550,531,0)
# target = Position(550,500,20)
agent_target_posn = sim_env.convert_to_agent_coordinates(0,target)
sensor_frame_posn = sim_env.convert_to_sensor_frame_coordinates(0, agent_target_posn)
yb = sim_env.create_agent_yolobox(0, target)
det = sa.create_detection(yb, True)
det_posn = det.get_position()

print(f"target: \t{target.get_cartesian_coordinates()}")
print(f"agent:  \t{agent_target_posn.get_cartesian_coordinates()}")

print(f"sensor_frame:\t{sensor_frame_posn.get_cartesian_coordinates()}")
print(f"detection: \t{det.get_cartesian_coordinates()}\t{det.get_angles()}")
print(f"{'-'*20}\nmapping back!")
print(f"detection: \t{det.get_cartesian_coordinates()}\t{det.get_angles()}")
sensor_frame_posn2 = sa.transform_from_det_coord(det)
print(f"sensor_frame:\t{sensor_frame_posn2.get_cartesian_coordinates()}")
agent_target_2 = sa.transform_from_frame_coord(sensor_frame_posn2)
print(f"agent2: \t{agent_target_2.get_cartesian_coordinates()}\t{agent_target_2.get_angles()}")
target_2 = sim_env.convert_from_agent_coordinates(0, agent_target_2)
print(f"target2:\t{target_2.get_cartesian_coordinates()}")





