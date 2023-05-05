
#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link

import collections
# from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
# from Scene import *
import json

# import pygame
# import numpy as np
import sys
# import time

from RigidBody import RigidBody
from Sensor import Sensor
from SensingAgent import SensingAgent
import pygame
import time

class Environment:
  def __init__(self,agent = None,
                    targets = []):
    self.agent = agent
    self.targets = targets
  
  def visible_targets(self):
    '''
    Determines visible targets in the vicinity of agent A and updates
    the agent A's tracker.

    Does not return
    '''
    pairs = []
    sortkey = lambda x: x[2]
    frame_id = "frame_"+str(len(self.agent.obj_tracker.layers))
    
    for target in self.targets:
      d = mfn.euclidean_dist(self.agent.get_sensor_origin(), target.get_origin())
      pairs.append((self.agent, target, d))
    pairs = sorted(pairs, key=sortkey)
    c = 0
    pl = []
    add_list = []
    while c < len(pairs):
      if pairs[c][2] > pairs[c][0].get_sensor_range():
        break
      if pairs[c][0].query_sensor_visible(pairs[c][1].get_origin()):
        add_list = self.notify_agent(pairs[c][1],frame_id, add_list)
        
      c+=1
    self.agent.obj_tracker.add_new_layer(add_list)
    self.agent.obj_tracker.process_layer(len(self.agent.obj_tracker.layers) - 1)

  def add_target(self,T):

    self.targets.append(T)
  
  def notify_agent(self, target, frame_id, add_list = []):
    dc = self.agent.sensor.transform_to_local_coord(target.get_origin())
    yb = sann.register_annotation(0, dc, frame_id)
    add_list.append(yb)
    return add_list