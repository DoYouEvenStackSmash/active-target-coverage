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
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment
import sys
import json
import pygame
import time

COLLISION_THRESHOLD = 10
VERBOSE = True
SAMPLE_RATE = 400
LALT = 256
LSHIFT = 1
LCTRL = 64
SPACE = 32
OFFT = 20
SPLINE_COUNT = 2
TRANSLATE = False


def draw_coordinate_frame(screen, sensor):
  '''
  Helper function for displaying the sensor field of view
  Does not return
  '''
  levels = 5
  coord_frame = sensor.get_visible_fov(levels)
  detect_frame = sensor.get_detectable_bounds(levels)
  for c in range(levels):
    for i in range(1,len(coord_frame[c])):
      pafn.frame_draw_line(screen, (coord_frame[c][i-1], coord_frame[c][i]), pafn.colors['white'])
      
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (sensor.get_origin(), endpoint), pafn.colors['white'])
  
  for endpoint in detect_frame[-1]:
    pafn.frame_draw_line(screen, (sensor.get_origin(), endpoint), pafn.colors['tangerine'])


def draw_all_normals(screen, rigid_body):
  '''
  Render function for all coordinate frames in a chain
  Does not return
  '''
  n = rigid_body.get_normals()
  pafn.frame_draw_line(screen, (rigid_body.get_center(), n[0]), pafn.colors['tangerine'])
  pafn.frame_draw_line(screen, (rigid_body.get_center(), n[1]), pafn.colors['yellow'])
  pafn.frame_draw_dot(screen, rigid_body.get_endpoint(), pafn.colors['white'])

def draw_all_links(screen, link):
  '''
  Render function for all polygon links in the chain
  Does not return
  '''
  points = link.get_points()
  pafn.frame_draw_polygon(screen, points, pafn.colors["red"])


def draw_rigid_body(screen, rigid_body):
  '''
  Wrapper for rendering all components of a rigid body
  '''
  draw_all_normals(screen, rigid_body)
  draw_all_links(screen, rigid_body)

def draw_sensing_agent(screen, sensing_agent):
  '''
  Wrapper for rendering all components of an agent
  '''
  exoskeleton,sensor = sensing_agent.get_components()
  draw_coordinate_frame(screen, sensor)
  draw_rigid_body(screen, exoskeleton)

def repeatable_environment_test(screen, sensing_agent, environment):
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  draw_sensing_agent(screen, environment.agent)
  pygame.display.update()
    
  step_size = 15
  destinations = []
  origin = (600,500)
  for i in range(25):
    x,y = origin
    # destinations.append((x, y - step_size * i))
    destinations.append((x - 300, y - step_size * i))
  destinations.reverse()
  for i in reversed(destinations):
    destinations.append(i)
  
  
  ptr = environment.targets[0].get_origin()
  pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  translation_path = destinations
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        if pygame.key.get_mods() == SPACE:
          
          continue
        elif pygame.key.get_mods() == LSHIFT:  # rotate relative
          for pt in translation_path[1:]:
            pafn.clear_frame(screen)
            sensing_agent.predict()
            curr_pt, pred_pt = sensing_agent.estimate_next_detection()
            if len(pred_pt):
              pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
              pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])
            pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
            draw_sensing_agent(screen, environment.agent)
            environment.targets[0].origin = pt
            environment.visible_targets()
            pygame.display.update()
            time.sleep(0.1)
          
          continue
        elif pygame.key.get_mods() == LALT: # estimate
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pafn.clear_frame(screen)
          rotation = sensing_agent.rotate_agent(p)
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
          
        elif pygame.key.get_mods() == LCTRL:
          e = environment.agent.export_tracks()
          f = open("out.json", "w")
          f.write(json.dumps(e, indent = 2))
          f.close()
          sys.exit()
          continue
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()


def repeatable_sensing_agent(screen, sensing_agent):
  draw_sensing_agent(screen, sensing_agent)
  pygame.display.update()
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        if pygame.key.get_mods() == SPACE:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pafn.clear_frame(screen)
          sensing_agent.translate_agent(p)
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
        elif pygame.key.get_mods() == LSHIFT:  # rotate relative
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pafn.clear_frame(screen)
          rotation = sensing_agent.rotate_agent(p)
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
        elif pygame.key.get_mods() == LALT: # estimate
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          visible = sensing_agent.is_visible(p)
          if visible:
            pafn.frame_draw_dot(screen, p, pafn.colors['green'])
          else:
            pafn.frame_draw_dot(screen, p, pafn.colors['red'])
          pygame.display.update()
          continue
        elif pygame.key.get_mods() == LCTRL:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          dc = sensing_agent.transform_to_local_bbox(p)
          print(f"original {p}\ndc {dc}")
          continue
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          detectable,flag = sensing_agent.is_detectable(p)
          if detectable:
            pafn.frame_draw_dot(screen, p, pafn.colors['cyan'])
          else:
            pafn.frame_draw_dot(screen, p, pafn.colors['red'])
          pygame.display.update()

  

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  sensing_agent = SensingAgent()
  ox,oy = 400,400
  scale = 3
  opts = [(ox - 10*scale, oy - 10*scale), (ox - 10*scale, oy + 10*scale), (ox + 30 * scale, oy)]
  mpt = gfn.get_midpoint(opts[0], opts[1])
  mpt2 = gfn.get_midpoint(mpt,opts[2])
  ap = Polygon(opts)
  rb = RigidBody(parent_agent=sensing_agent, ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_link = ap)
  sensor = Sensor(parent_agent = sensing_agent)
  sensing_agent.exoskeleton = rb
  sensing_agent.sensor = sensor
  sensing_agent.obj_tracker = ObjectTrackManager()
  target = Target((500,550))
  environment = Environment(sensing_agent, [target])
  # repeatable_sensing_agent(screen, sensing_agent)
  repeatable_environment_test(screen, sensing_agent, environment)




if __name__ == '__main__':
  main()

