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

def adjust_angle(theta):
  ''' adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]'''
  if theta > np.pi:
    theta = theta + -2 * np.pi
  elif theta < -np.pi:
    theta = theta + 2 * np.pi
  
  return theta
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
    
  step_size = 25
  vert_destinations = []
  horiz_destinations = []
  origin = (600,500)
  for i in range(25):
    x,y = origin
    # destinations.append((x, y - step_size * i))
    vert_destinations.append((x, y - step_size * i))
    horiz_destinations.append((x - step_size * i, y))

  vert_destinations.reverse()
  # horiz_destinations.reverse()
  for i in reversed(vert_destinations):#horiz_destinations):
    vert_destinations.append(i)
    horiz_destinations.append(i)
  
  
  ptr = environment.targets[0].get_origin()
  pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  translation_path = vert_destinations
  # translation_path
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        if pygame.key.get_mods() == SPACE:
          
          continue
        elif pygame.key.get_mods() == LSHIFT:  # rotate relative
          for pt in translation_path[1:]:
            pafn.clear_frame(screen)
            # sensing_agent.predict()
            # print(sensing_agent.estimate_next_rotation())
            curr_pt, pred_pt = sensing_agent.estimate_next_detection()
            sensing_agent.estimate_next_rotation()
            if len(pred_pt):
              
              # print((curr_pt,pred_pt))
              pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
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
          # sys.exit()
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          dc = sensing_agent.transform_to_local_bbox(p)
          print(f"original {p}\ndc {dc}")
          continue

        elif pygame.key.get_mods() == LCTRL:
          pafn.clear_frame(screen)
          draw_sensing_agent(screen, environment.agent)
          curr_pt, pred_pt = sensing_agent.estimate_next_detection()
          if len(pred_pt):
            # print((curr_pt,pred_pt))
            pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
            pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
            pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])
          pygame.display.update()
          continue
          # pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          # pts.append(p)
          # dc = sensing_agent.transform_to_local_bbox(p)
          pafn.clear_frame(screen)
          orig_theta = sensing_agent.get_fov_theta()
          rotation = sensing_agent.rotate_agent(p)
          
          new_theta = sensing_agent.get_fov_theta()
          print(new_theta - orig_theta)
          dc = sensing_agent.transform_to_local_bbox(p)
          # print(f"displacement: {rotation / sensing_agent.get_fov_width() * 100}")
          # dc = sensing_agent.transform_to_local_bbox(p)
          print(f"original {p}\ndc {dc}")
          print(new_theta)
          
          # sensing_agent.obj_tracker.add_angular_displacement(0, -(new_theta - orig_theta))
          
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
         
def repeatable_step_test(screen, sensing_agent, environment):
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  draw_sensing_agent(screen, environment.agent)
  pygame.display.update()
  
  step_size = 25
  vert_destinations = []
  horiz_destinations = []
  origin = (600,500)
  for i in range(25):
    x,y = origin
    # destinations.append((x, y - step_size * i))
    vert_destinations.append((x, y - step_size * i))
    horiz_destinations.append((x - step_size * i, y))
  
  vert_destinations.reverse()
  horiz_destinations.reverse()
  for i in reversed(vert_destinations):#horiz_destinations):
    vert_destinations.append(i)
    horiz_destinations.append(i)
  # horiz_destinations.reverse()
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        if pygame.key.get_mods() == SPACE:
          continue
        elif pygame.key.get_mods() == LSHIFT:  # rotate relative
          e = sensing_agent.export_tracks()
          f = open("out.json", "w")
          f.write(json.dumps(e,indent=2))
          f.close()
          sys.exit()
          continue
        elif pygame.key.get_mods() == LALT: # estimate
            curr_pt, pred_pt = sensing_agent.estimate_next_detection()

            if len(pred_pt):
              # print((curr_pt,pred_pt))
              pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
              pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
              pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])
            pygame.display.update()
            continue
        elif pygame.key.get_mods() == LCTRL:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pafn.clear_frame(screen)
          orig = sensing_agent.get_fov_theta()
          rotation = sensing_agent.rotate_agent(p)
          new_theta = sensing_agent.get_fov_theta()
          print("checking...")
          print(new_theta - orig)
          print(rotation)
          # sensing_agent.obj_tracker.add_angular_displacement(0,orig - new_theta)
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
          # pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          translation_path = []
          translation_path = gfn.lerp_list(environment.targets[0].get_origin(), p, 10)
          # translation_path = horiz_destinations
          for pt in translation_path[1:]:
            pafn.clear_frame(screen)
            # curr_pt, pred_pt = sensing_agent.estimate_next_detection()
            # specify displacement
            est_rotation = ()
            est_rotation = sensing_agent.estimate_next_rotation()
            
            pred_rotation = sensing_agent.exoskeleton.get_relative_rotation(pt)
            if len(est_rotation):
              
              est_rotation = est_rotation[0]
              print(est_rotation)
              rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
              sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
              sensing_agent.exoskeleton.rel_theta += rotation
              # print(f"sensing: {sensing_agent.exoskeleton.rel_theta}")
              # sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
              # draw_sensing_agent(screen, sensing_agent)
              # pygame.display.update()
              time.sleep(0.01)

              # rotation = sensing_agent.apply_rotation_to_agent(-est_rotation)
              # sensing_agent.exoskeleton.rel_theta += rotation
              # sensing_agent.obj_tracker.add_angular_displacement(0, est_rotation)
              # draw_sensing_agent(screen, sensing_agent)
              # pygame.display.update()
              # time.sleep(0.2)
            # sensing_agent.obj_tracker.add_angular_displacement(0, -rotation)


            # print(f"est:\t{est_rotation}\npred:\t{pred_rotation}")
            curr_pt, pred_pt = sensing_agent.estimate_next_detection()
            if len(pred_pt):
              # print((curr_pt,pred_pt))
              pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
              pafn.frame_draw_dot(screen, pred_pt, pafn.colors["tangerine"])
              pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])
            # npt = sensing_agent.estimate_next_rotation()
            # if len(npt):
            #   npt = sensing_agent.transform_from_local_coord(npt[0], npt[1])
            #   pafn.frame_draw_dot(screen, npt, pafn.colors["cyan"])
            

            draw_sensing_agent(screen, sensing_agent)
            pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
            environment.targets[0].origin = pt
            environment.visible_targets()
            pygame.display.update()
            time.sleep(0.1)
            # sensing_agentestimate_next_rotation()
          continue

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
          fc = sensing_agent.transform_from_local_coord(dc[0],dc[1])
          print(f"original {p}\tdc {dc}\tfc {fc}")
          continue
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          dc = sensing_agent.transform_to_local_bbox(p)
          detectable,flag = sensing_agent.is_rel_detectable((dc[0],dc[1]))
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
  sensing_agent.obj_tracker.parent_agent = sensing_agent
  target = Target((500,550))
  environment = Environment(sensing_agent, [target])
  # repeatable_sensing_agent(screen, sensing_agent)
  # repeatable_environment_test(screen, sensing_agent, environment)
  repeatable_step_test(screen, sensing_agent, environment)




if __name__ == '__main__':
  main()

