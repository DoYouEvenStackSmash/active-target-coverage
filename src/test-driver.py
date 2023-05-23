#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
import pygame

import collections

from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment

import sys
import json
import pygame
import pygame.gfxdraw
import time
import os

from drawing_functions import *

def mouse_test(screen, sensing_agent, environment):
  pt = None
  last_pt = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        e = sensing_agent.export_tracks()
        f = open("out.json", "w")
        f.write(json.dumps(e, indent=2))
        f.close()
        sys.exit()
      pt = pygame.mouse.get_pos()
      if last_pt == pt:
        continue
      
      last_pt = pt
      print(pt)
    
      pafn.clear_frame(screen)

      est_rotation,est_translation = sensing_agent.estimate_pose_update()
      print(f"estimated: {est_rotation}")

      if est_rotation != None:

        rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
        sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
        sensing_agent.exoskeleton.rel_theta += rotation

      if est_translation != None:
        
        translation = sensing_agent.apply_translation_to_agent(est_translation)
        sensing_agent.obj_tracker.add_linear_displacement(-translation, 0)

      curr_pt, pred_pt = sensing_agent.estimate_next_detection()
      
      if len(pred_pt):
        
        pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
        pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
        pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])

      for k,sensing_agent in environment.agents.items():
        draw_sensing_agent(screen, sensing_agent)
      pafn.frame_draw_dot(screen, pt, pafn.colors["lawngreen"])
      environment.targets[0].origin = pt
      environment.visible_targets()
      pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
      pygame.display.update()
  

def init_sensing_agent(sensing_agent = SensingAgent(),origin = (0,0), _id = 0, orientation = (0,0)):
  ox,oy = origin
  scale = 2
  opts = [(ox - 10*scale, oy - 10*scale), (ox - 10*scale, oy + 10*scale), (ox + 30 * scale, oy)]
  # print(opts)
  
  mpt = gfn.get_midpoint(opts[0], opts[1])
  mpt2 = gfn.get_midpoint(mpt,opts[2])
  ap = Polygon(opts)
  rb = RigidBody(parent_agent=sensing_agent, ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_link = ap)
  sensor = Sensor(parent_agent = sensing_agent)
  sensor.fov_width = np.pi / 4
  
  sensing_agent.exoskeleton = rb
  sensing_agent.centered_sensor = sensor
  sensing_agent.obj_tracker = ObjectTrackManager()
  sensing_agent.obj_tracker.parent_agent = sensing_agent
  sensing_agent._id = _id
  rotation = sensing_agent.rotate_agent(orientation)
  return sensing_agent

def main():
  ox,oy = 500,500
  target = Target((600, 650),_id=41)
  
  environment = Environment()
  sensing_agent = init_sensing_agent(SensingAgent(), (ox,oy), "A")
  environment.add_target(target)
  environment.agents["A"] = sensing_agent
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pafn.clear_frame(screen)
  mouse_test(screen, sensing_agent, environment)

if __name__ == '__main__':
  main()