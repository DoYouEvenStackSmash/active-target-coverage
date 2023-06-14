#!/usr/bin/python3
"""
A test for initializing targets with paths
"""
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

target_path = [(1, 1), (2, 2)]

t = init_target(path=target_path)
# steps a target along its path
print(t.get_position())
t.step()

print(t.get_position())
t.step()

print(t.get_position())
