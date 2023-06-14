#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

"""
A test which initializes environment objects
"""
sa = init_sensing_agent()
t = init_target()
env = init_environment(sensing_agents={sa._id: sa}, targets=[t])
print(env)
