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
            rotation = gfn.lerp_list(Tlist[0].get_origin(), pred, 10)
            start = 1
          
          # incrementally rotate the agent
          for p in rotation[start:]:
            pafn.clear_frame(screen)
            pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
            pafn.frame_draw_dot(screen, Tlist[0].get_origin(), pafn.colors["green"])
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

def predict_coverage(A,screen):
  estimates = A.predict_targets_covered()
  for i in estimates:
    pred = A.transform_from_local_coord(i[1][0],i[1][1])
    curr = A.transform_from_local_coord(i[0][0],i[0][1])
    if len(pred) > 0:
      if screen != None:
        pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
        pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])

def repeatable_test(screen, A, T):
  Tlist = [T]
  Alist = [A]
  T.origin = (500,500)
  step_size = 15
  destinations = []
  origin = (500,500)
  for i in range(25):
    x,y = origin
    # destinations.append((x, y - step_size * i))
    destinations.append(((x - step_size * i), y))
  destinations.reverse()
  for i in reversed(destinations):
    destinations.append(i)
  for t in range(len([Tlist])):
    # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
    # Tlist[t].origin = pt
    ptr = Tlist[t].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  
  
  translation_path = destinations
  print("running\n")

  rotate_counter = 0
  for pt in translation_path[1:]:
    
    pafn.clear_frame(screen)
    # T.origin = pt
    for a in range(len(Alist)):
      A = Alist[a]
      predict_coverage(A,screen)
      adjust_for_coverage(A, Tlist,screen)
    # time.sleep(.05)
      draw_coordinate_frame(screen, A)
    pygame.display.update()
    time.sleep(.01)
    theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
    for t in range(len([Tlist])):
      # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
      Tlist[t].origin = pt
      ptr = Tlist[t].get_origin()
      pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
    pygame.display.update()
    for a in range(len(Alist)):
      visible_targets(Alist[a], Tlist)
    time.sleep(0.2)
    
  
  print(f"num_steps:{len(translation_path)}")
  print(f"rotations:{rotate_counter}")
  for A in Alist:
    export_tracks(screen, A)
      # pygame.display.update()

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  A = Agent([400,400], [np.pi / 2, 200, np.pi / 4], obj_tracker = ObjectTrackManager())
  T = Target((500,550))
  repeatable_test(screen, A, T)#,T2])

if __name__ =='__main__':
  main()
  