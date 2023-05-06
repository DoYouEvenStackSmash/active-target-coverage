
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
    dc = self.transform_to_local_coord(target.get_origin())
    yb = sann.register_annotation(0, dc, frame_id)
    add_list.append(yb)
    return add_list
  
  def transform_to_local_coord(self,target):
    '''
    Calculates detection coordinates relative to Sensor
    returns a Yolo Formatted bbox
    '''
    tar_theta, tar_r = mfn.car2pol(self.agent.exoskeleton.ref_center, target)
    org_theta = self.agent.fov_theta
    if org_theta < 0:
      org_theta = 2 * np.pi + org_theta
 
    lh = org_theta + self.agent.sensor.fov_width / 2
    rh = org_theta - self.agent.sensor.fov_width / 2
    if tar_theta < rh:
      tar_theta = tar_theta + 2 * np.pi
    
    ratio = (lh - tar_theta) / (lh - rh)
    
    x = Sensor.WINDOW_WIDTH * ratio
    y = tar_r
    w = 1
    h = 1
    return [x,y,w,h]
  
  def transform_from_local_coord(self, x, y, w=1, h=1):
    '''
    Transforms a bbox from Sensor local coordinates to world coordinates
    returns a Point
    '''
    org_theta = mfn.correct_angle(self.agent.fov_theta)
    
    rh = org_theta - self.agent.sensor.fov_width / 2
    lh = org_theta + self.agent.sensor.fov_width / 2
    theta = (x / Sensor.WINDOW_WIDTH) * self.agent.sensor.fov_width
    # print(f"is_not_visible: {lh}:{theta}:{rh}")
    ratio = theta - 0.5
    theta = lh - theta
    theta = mfn.correct_angle(theta)
    
    r =  y
    return mfn.pol2car(self.agent.get_origin(), r, theta)