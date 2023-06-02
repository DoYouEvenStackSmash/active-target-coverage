#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

target_path=[Position(1,1,0),Position(2,2,0)]

t = init_target(path=target_path)
# steps a target along its path
print(t.get_position())
t.step()

print(t.get_position())
t.step()

print(t.get_position())

