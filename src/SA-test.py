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
import os

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
# module = sys.modules['__main__']
# path, name = os.path.split(module.__file__)
# path = os.path.join(path, 'jackal.png')

# img = pygame.image.load(path)


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
  # for i in range(1, len(detect_frame[-1])):

  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (sensor.get_origin(), endpoint), pafn.colors['white'])
  spts = []
  epts = []
  for endpoint in detect_frame[-1]:
    pafn.frame_draw_line(screen, (sensor.get_origin(), endpoint), pafn.colors['tangerine'])
    theta, r = mfn.car2pol(sensor.get_origin(), endpoint)
    r2 = Sensor.TOLERANCE * r
    r *= 1 - Sensor.TOLERANCE
    pt2 = mfn.pol2car(sensor.get_origin(), r2, theta)
    pt = mfn.pol2car(sensor.get_origin(), r, theta)
    
    epts.append(pt)
    spts.append(pt2)
  
  # for i in range(1, len(epts)):
  #   pafn.frame_draw_bold_line(screen, (epts[i-1], epts[i]), pafn.colors['tangerine'])
  #   pafn.frame_draw_bold_line(screen, (spts[i-1], spts[i]), pafn.colors['tangerine'])



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
    
  step_size = 35
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

          pafn.clear_frame(screen)
          orig_theta = sensing_agent.get_fov_theta()
          rotation = sensing_agent.rotate_agent(p)
          
          new_theta = sensing_agent.get_fov_theta()
          print(new_theta - orig_theta)
          dc = sensing_agent.transform_to_local_bbox(p)

          print(f"original {p}\ndc {dc}")
          print(new_theta)
          
          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
         
def repeatable_step_test(screen, sensing_agent, environment):
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  draw_sensing_agent(screen, environment.agent)
  pygame.display.update()
  pts = []
  step_size = 21
  vert_destinations = []
  horiz_destinations = []
  origin = (600,500)
  for i in range(25):
    x,y = origin
    
    vert_destinations.append((x, y - step_size * i))
    horiz_destinations.append((x - step_size * i, y))
  
  vert_destinations.reverse()
  horiz_destinations.reverse()
  for i in reversed(vert_destinations):#horiz_destinations):
    vert_destinations.append(i)
    horiz_destinations.append(i)
  
  vert_destinations = horiz_destinations
  
  step_sz = 25
  l1 = gfn.lerp_list(horiz_destinations[0],horiz_destinations[step_sz//2],step_sz)
  l2 = gfn.lerp_list(horiz_destinations[step_sz//2],horiz_destinations[-1],step_sz)
  m1 = []
  for i in range(step_sz):
    m1.append(gfn.lerp(l1[i],l2[i], i / step_sz))
  # horiz_destinations = m1
  rev_vert = []
  for i in reversed(vert_destinations):
    rev_vert.append(i)
  rev_m1 = []
  for i in reversed(m1):
    rev_m1.append(i)
  arrs = [vert_destinations, rev_vert, m1, rev_m1]
  idx = 1
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
          
          vert_destinations.reverse()
          idx ^= 1

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

          draw_sensing_agent(screen, sensing_agent)
          pygame.display.update()
          continue
          
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          
          p = pygame.mouse.get_pos()
          pts.append(p)
          # pafn.frame_draw_dot(screen, p, pafn.colors["magenta"])
          # pygame.display.update()
          for idx in range(len(arrs)):
            translation_path = arrs[idx]
            # for p in translation_path:
            #   pafn.frame_draw_dot(screen, p, pafn.colors['cyan'])
            #   pygame.display.update()
            #   time.sleep(0.005)
            for i in range(1, len(translation_path)):
              pt = translation_path[i]
              pafn.clear_frame(screen)

              est_rotation = ()
              pred_rotation = sensing_agent.exoskeleton.get_relative_rotation(pt)
              print(f"predicted: {pred_rotation}\t",end="")
              est_rotation,est_translation = sensing_agent.estimate_next_rotation()
              print(f"estimated: {est_rotation}")
              # global img
              # img = img0
              # rect = img.get_rect()
              if est_rotation != None:
                # est_rotation = est_rotation[0]
                # deg = est_rotation * (180 / np.pi)
                # img = pygame.transform.rotozoom(img, deg, 1)
                # rect = img.get_rect()
                # rect.center = sensing_agent.get_center()
                # print(est_rotation)
                rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
                sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
                sensing_agent.exoskeleton.rel_theta += rotation

              if est_translation != None:
                
                translation = sensing_agent.apply_translation_to_agent(est_translation)
                sensing_agent.obj_tracker.add_linear_displacement(-translation, 0)
                
                
              # rect.center = sensing_agent.get_center()
              # screen.blit(img, rect)
                
                # print(est_rotation)

              # draw_sensing_agent(screen, sensing_agent)
              # pygame.display.update()
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
              time.sleep(0.2)
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

def repeatable_multiagent_test(screen, environment):
  time.sleep(3)
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  # sensing_agents = [v for k,v in environment.agents.items()]
  # for sensing_agent in sensing_agents:
  #   draw_sensing_agent(screen, sensing_agent)
  # pygame.display.update()
  for k,sensing_agent in environment.agents.items():
      draw_sensing_agent(screen, sensing_agent)
  pts = []
  step_size = 21
  vert_destinations = []
  horiz_destinations = []
  origin = (600,500)
  for i in range(25):
    x,y = origin
    
    vert_destinations.append((x, y - step_size * i))
    horiz_destinations.append((x - step_size * i, y))
  
  vert_destinations.reverse()
  horiz_destinations.reverse()
  for i in reversed(vert_destinations):#horiz_destinations):
    vert_destinations.append(i)
    horiz_destinations.append(i)
  
  vert_destinations = horiz_destinations
  
  step_sz = 25
  l1 = gfn.lerp_list(horiz_destinations[0],horiz_destinations[step_sz//2],step_sz)
  l2 = gfn.lerp_list(horiz_destinations[step_sz//2],horiz_destinations[-1],step_sz)
  m1 = []
  for i in range(step_sz):
    m1.append(gfn.lerp(l1[i],l2[i], i / step_sz))
  # horiz_destinations = m1
  rev_vert = []
  for i in reversed(vert_destinations):
    rev_vert.append(i)
  rev_m1 = []
  for i in reversed(m1):
    rev_m1.append(i)
  arrs = [vert_destinations, rev_vert, m1, rev_m1]
  idx = 1
  for idx in range(len(arrs)):
    translation_path = arrs[idx]

    for i in range(1, len(translation_path)):
      pt = translation_path[i]
      pafn.clear_frame(screen)
      print(pt)
      # continue
      for _id in environment.agents:
        agent = environment.agents[_id]
        pred_rotation = agent.exoskeleton.get_relative_rotation(pt)
        est_rotation, est_translation = None,None
        est_rotation,est_translation = agent.estimate_next_rotation()
        
        if est_rotation != None:
          rotation = 0
          rotation = agent.apply_rotation_to_agent(est_rotation)
          agent.obj_tracker.add_angular_displacement(0, -est_rotation)
          agent.exoskeleton.rel_theta += rotation

        if est_translation != None:
          translation = 0
          translation = agent.apply_translation_to_agent(est_translation)
          agent.obj_tracker.add_linear_displacement(-translation, 0)

        curr_pt, pred_pt = agent.estimate_next_detection()
        # # environment.agents[_id] = sensing_agent
        if len(pred_pt):
          pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
          pafn.frame_draw_dot(screen, pred_pt, pafn.colors["tangerine"])
          pafn.frame_draw_line(screen, (curr_pt, pred_pt),pafn.colors["white"])
        # draw_sensing_agent(screen, agent)
        # draw_sensing_agent(screen, sensing_agent)
      for k,sensing_agent in environment.agents.items():
        draw_sensing_agent(screen, sensing_agent)
      pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
      environment.targets[0].origin = pt
      environment.visible_targets()
      
      pygame.display.update()
      
      time.sleep(0.09)

def init_sensing_agent(sensing_agent = SensingAgent(),origin = (0,0), _id = 0, orientation = (0,0)):
  ox,oy = origin
  scale = 2
  opts = [(ox - 10*scale, oy - 10*scale), (ox - 10*scale, oy + 10*scale), (ox + 30 * scale, oy)]
  mpt = gfn.get_midpoint(opts[0], opts[1])
  mpt2 = gfn.get_midpoint(mpt,opts[2])
  ap = Polygon(opts)
  rb = RigidBody(parent_agent=sensing_agent, ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_link = ap)
  sensor = Sensor(parent_agent = sensing_agent)
  sensor.fov_width = np.pi / 4
  sensor.fov_radius = 250
  sensing_agent.exoskeleton = rb
  sensing_agent.sensor = sensor
  sensing_agent.obj_tracker = ObjectTrackManager()
  sensing_agent.obj_tracker.parent_agent = sensing_agent
  sensing_agent._id = _id
  rotation = sensing_agent.rotate_agent(orientation)
  return sensing_agent

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  
  ox,oy = 400,400
  ids = ["A", "B"]  
  origins = [(800,300), (300,600)]
  orientations = [(200,400), (400,400)]
  environment = Environment()

  # for i in range(2):
  #   environment.agents[ids[i]] = 
  Agents = [init_sensing_agent(SensingAgent(), origins[i], ids[i], orientations[i]) for i in range(2)]
  Agents[1].sensor.fov_radius = 150
  target = Target((500,550))
  
  
  environment.agents = {ids[0] : Agents[0], ids[1] : Agents[1]}
  environment.add_target(target)
  # environment.targets.append(target)
  # opts = [(ox - 10*scale, oy - 10*scale), (ox - 10*scale, oy + 10*scale), (ox + 30 * scale, oy)]
  # mpt = gfn.get_midpoint(opts[0], opts[1])
  # mpt2 = gfn.get_midpoint(mpt,opts[2])
  # ap = Polygon(opts)
  # rb = RigidBody(parent_agent=sensing_agent, ref_origin = mpt, ref_center = mpt2, endpoint = opts[2], rigid_link = ap)
  # sensor = Sensor(parent_agent = sensing_agent)
  # sensor.fov_width = np.pi / 4
  # sensor.fov_radius = 300
  # sensing_agent.exoskeleton = rb
  # sensing_agent.sensor = sensor
  # sensing_agent.obj_tracker = ObjectTrackManager()
  # sensing_agent.obj_tracker.parent_agent = sensing_agent
  # rotation = sensing_agent.rotate_agent((600,600))
  
  # environment = Environment(sensing_agent, [target])
  # img.convert()
  # rect = img.get_rect()
  # rect.center = sensing_agent.get_center()
  # pygame.display.update()
  # repeatable_sensing_agent(screen, sensing_agent)
  # repeatable_environment_test(screen, sensing_agent, environment)
  repeatable_multiagent_test(screen, environment)
  # repeatable_step_test(screen, sensing_agent, environment)




if __name__ == '__main__':
  main()

