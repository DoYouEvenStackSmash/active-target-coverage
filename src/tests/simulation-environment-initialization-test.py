#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

"""
Initializes environment objects
"""
sa = init_sensing_agent()
t = init_target()
sim_env = init_sim_environment(sensing_agents={sa._id: sa}, targets=[t])
print(sim_env)
