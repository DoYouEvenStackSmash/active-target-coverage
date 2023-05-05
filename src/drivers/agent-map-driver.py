#!/usr/bin/python3
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from Agent import *

from PIL import Image, ImageDraw
import collections
from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from Scene import *
import json

import pygame
import numpy as np
import sys
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



def drive(screen, origin, pt):
  rad, r = MathFxns.car2pol(origin, pt)
  if rad < np.pi/2 and rad > -np.pi/2:
    print("turn right")
    pafn.frame_draw_line(screen, (origin, pt), pafn.colors["green"])
  else:
    print("turn left")
    pafn.frame_draw_line(screen, (origin, pt), pafn.colors["red"])

def predict_coverage(A,screen):
  estimates = A.predict_targets_covered()
  for i in estimates:
    pred = A.transform_from_local_coord(i[1][0],i[1][1])
    curr = A.transform_from_local_coord(i[0][0],i[0][1])
    if len(pred) > 0:
      if screen != None:
        pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
        pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])

def adjust_for_coverage(A, Tlist = [], screen = None):
  
  estimates = A.predict_targets_covered()
  j = 0
  while j < len(estimates):
    i = estimates[j]
    indicator, err_type = A.is_detectable(i[1])
    if indicator:
      j+=1
      continue
    else:
      pred = A.transform_from_local_coord(i[1][0],i[1][1])
      curr = A.transform_from_local_coord(i[0][0],i[0][1])

      if len(pred) > 0:
        if screen != None:
          pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])
        if err_type == Agent.ANGULAR:
          start = 0
          rotation = []

          if screen == None:
            A.rotate_sensor(pred)
            estimates = A.predict_targets_covered()
            j = 0
            break
          else:
            rotation = gfn.lerp_list(Tlist[0], pred, 30)
            start = 1
          
          # incrementally rotate the agent
          for p in rotation[start:]:
            pafn.clear_frame(screen)
            pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
            pafn.frame_draw_dot(screen, Tlist[0], pafn.colors["green"])
            # pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
            A.rotate_sensor(p)
            draw_coordinate_frame(screen, A)
            pygame.display.update()
            time.sleep(0.01)
          estimates = A.predict_targets_covered()
          j = 0
          break
        elif err_type == Agent.RANGE:
          A.translate_sensor(pred)
          estimates = A.predict_targets_covered()
          j = 0
          break
        # recompute estimates
      if len(curr) > 0:
        if screen != None:
          pafn.frame_draw_dot(screen, curr, pafn.colors['red'])

def agent_map(screen, A):
  rel_max_x = 100
  rel_min_x = 0
  abs_max_x = 1000
  abs_min_y = 0
  agent_origin = mfn.pol2car(A.origin, A.fov_radius, A.fov_theta)
  origin = (500,500)
  draw_coordinate_frame(screen, A)
  pygame.display.update()
  crosshairs = [[(485,500), (515,500)], [(500,485), (500, 515)]]
  box = [(100,100),(100,900), (900,900), (900,100)]
  frame_id = lambda A :"frame_"+str(len(A.obj_tracker.layers))
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        if pygame.key.get_mods() == LCTRL:
          e = A.export_tracks()
          f = open("out.json", "w")
          f.write(json.dumps(e, indent = 2))
          f.close()
          sys.exit()
        # elif pygame.key.get_mods() == LSHIFT:  
        if pygame.key.get_mods() == LALT:
          estimations = A.predict_targets_covered()
          pafn.clear_frame(screen)
          for i in estimations:
            
            pred_abs_x = (i[1][0] / 100) * abs_max_x
            pred_abs_y = abs_max_x / 2

            curr_abs_x = (i[0][0] / 100) * abs_max_x
            curr_abs_y = abs_max_x / 2
            curr = (curr_abs_x, curr_abs_y)
            pred = (pred_abs_x, pred_abs_y)
            pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
            pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])
          
            # pred = A.transform_from_local_coord(i[1][0],i[1][1])
            # curr = A.transform_from_local_coord(i[0][0],i[0][1])

        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          # adjust_for_coverage(A, [p], screen)
          A.adjust_for_coverage()
          p = pygame.mouse.get_pos()
          pafn.frame_draw_dot(screen, p, pafn.colors["tangerine"])
          x,y = p
          sign_x_disp = (x - 500) / 500

          # theta, radius = mfn.car2pol(origin, p)

          rel_x = mfn.pol2car(A.origin, A.fov_radius, A.fov_theta)
          rel_x = (x / abs_max_x) * rel_max_x
          rel_y = A.fov_radius
          p = A.transform_from_local_coord(rel_x, rel_y,1,1)
          pafn.frame_draw_dot(screen, p, pafn.colors["green"])
          p = A.transform_to_local_coord(p)
          add_list = []
          yb = sann.register_annotation(0, p, frame_id(A))
          add_list.append(yb)
          A.obj_tracker.add_new_layer(add_list)
          A.obj_tracker.process_layer(len(A.obj_tracker.layers) - 1)
          draw_coordinate_frame(screen, A)
          
          
        pafn.frame_draw_polygon(screen, box, pafn.colors['indigo'])
        for c in crosshairs:
          pafn.frame_draw_line(screen, c, pafn.colors['magenta'])
        # pafn.frame_draw_polygon(screen, crosshairs, pafn.colors['magenta'])
        pygame.display.update()

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  A = Agent([700,700], [np.pi / 2, 200, np.pi / 4], obj_tracker = ObjectTrackManager())
  # T = Target((500,550))
  agent_map(screen, A)#,T2])

if __name__ =='__main__':
  main()
  