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
from Target import Target
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

def predict_coverage(A,screen):
  estimates = A.predict_targets_covered()
  for i in estimates:
    pred = A.transform_from_local_coord(i[1][0],i[1][1])
    curr = A.transform_from_local_coord(i[0][0],i[0][1])
    if len(pred) > 0:
      if screen != None:
        pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
        pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])

# def adjust_for_coverage(A, Tlist = [], screen = None):
  
#   estimates = A.predict_targets_covered()
#   j = 0
#   while j < len(estimates):
#     i = estimates[j]
#     indicator, err_type = A.is_detectable(i[1])
#     if indicator:
#       j+=1
#       continue
#     else:
#       pred = A.transform_from_local_coord(i[1][0],i[1][1])
#       curr = A.transform_from_local_coord(i[0][0],i[0][1])

#       if len(pred) > 0:
#         if screen != None:
#           pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])
#         if err_type == Agent.ANGULAR:
#           start = 0
#           rotation = []

#           if screen == None:
#             A.rotate_sensor(pred)
#             estimates = A.predict_targets_covered()
#             j = 0
#             break
#           else:
#             rotation = gfn.lerp_list(Tlist[0].get_origin(), pred, 30)
#             start = 1
          
#           # incrementally rotate the agent
#           for p in rotation[start:]:
#             pafn.clear_frame(screen)
#             pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
#             pafn.frame_draw_dot(screen, Tlist[0].get_origin(), pafn.colors["green"])
#             # pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
#             A.rotate_sensor(p)
#             draw_coordinate_frame(screen, A)
#             pygame.display.update()
#             time.sleep(0.01)
#           estimates = A.predict_targets_covered()
#           j = 0
#           break
#         elif err_type == Agent.RANGE:
#           A.translate_sensor(pred)
#           estimates = A.predict_targets_covered()
#           j = 0
#           break
#         # recompute estimates
#       if len(curr) > 0:
#         if screen != None:
#           pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
      


def moving_target(screen, Alist, Tlist):
  

  last_posn = (0,0)
  for A in Alist:
    draw_coordinate_frame(screen, A)    
  for t in range(len(Tlist)):
    ptr = Tlist[t].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  pygame.display.update()

  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        
        # Export the tracks
        if pygame.key.get_mods() == LCTRL:
          for A in Alist:
            export_tracks(screen, A)
          pygame.display.update()
          time.sleep(5)
          sys.exit()
        
        # rotate the sensor
        elif pygame.key.get_mods() == LSHIFT:  # rotate
          pafn.clear_frame(screen)
          for A in Alist:
            A.rotate_sensor(pygame.mouse.get_pos())
            draw_coordinate_frame(screen, A)
          for T in Tlist:
            pafn.frame_draw_dot(screen, T.get_origin(), pafn.colors["green"])
          pygame.display.update()
        
        # estimate the next position in the track
        elif pygame.key.get_mods() == LALT: # estimate
          pafn.clear_frame(screen)
          print(f"{'-'*100}")
          
          # estimate the coverage of each agent and draw the coordinate plane
          for A in Alist:
            predict_coverage(A, screen)
            # A.adjust_for_coverage()
            draw_coordinate_frame(screen, A)
          # print(f"{'='*100}")
          # draw the targets
          for T in Tlist:
            pafn.frame_draw_dot(screen, T.get_origin(), pafn.colors["green"])
          pygame.display.update()
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          last_posn = p
          translation_path = []
          translation_path = gfn.lerp_list(Tlist[0].get_origin(), p, 2)
          print("running\n")

          rotate_counter = 0
          for pt in translation_path[1:]:
            
            pafn.clear_frame(screen)
            # T.origin = pt
            for a in range(len(Alist)):
              # adjust_for_coverage(A, Tlist, screen)
              A.adjust_for_coverage()
            # time.sleep(.05)
              draw_coordinate_frame(screen, A)
            theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
            for t in range(len(Tlist)):
              Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
              ptr = Tlist[t].get_origin()
              pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
            
            for a in range(len(Alist)):
              visible_targets(Alist[a], Tlist)

            pygame.display.update()
          
          print(f"num_steps:{len(translation_path)}")
          print(f"rotations:{rotate_counter}")


def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  
  A = Agent([400,400], [0, 200, np.pi / 4], obj_tracker = ObjectTrackManager())
  # A.color = pafn.
  B = Agent([600,600], [np.pi/8, 200, np.pi / 6], obj_tracker = ObjectTrackManager())
  T = Target((500,550))
  T2 = Target((550,560))
  
  # print(A.obj_tracker.get_layer())
  moving_target(screen, [A], [T])#,T2])
  # moving_target(screen, [A, B], [T, T2])

if __name__ =='__main__':
  main()
  