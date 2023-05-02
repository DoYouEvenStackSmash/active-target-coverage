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

def estimate_next_detection(otm):
  curr_pt, pred_pt = (),()
  if otm.active_tracks != None and len(otm.active_tracks):
    print(f"active tracks: {len(otm.active_tracks)}")
    trk = otm.active_tracks[0]
    pred_pt = trk.predict_next_box()
    curr_pt = trk.get_last_detection()
  return (curr_pt, pred_pt)

def tracker_testing(screen):
  frame_id = 0
  o = ObjectTrackManager()
  origin = (500,500)
  conv_pt = lambda pt,opt: (pt[0] + opt[0], pt[1] + opt[1])
  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        if pygame.key.get_mods() == LCTRL:
        
          sys.exit()
        elif pygame.key.get_mods() == LALT:
          pafn.clear_frame(screen)
          cpt, pt = estimate_next_detection(o)
          
          if len(cpt) > 0:
            cpt = conv_pt(cpt, origin)
            pt = conv_pt(pt, origin)
            pafn.frame_draw_dot(screen, cpt, pafn.colors['yellow'])
            pafn.frame_draw_line(screen, (pt,cpt), pafn.colors["white"])
          if len(pt) > 0:
            pafn.frame_draw_dot(screen, pt, pafn.colors['red'])
          pygame.display.update()
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          add_list = []
          p = pygame.mouse.get_pos()

          x,y = p
          x = x - 500
          y = y - 500
          dc = [x,y,1,1]
          yb = sann.register_annotation(0, dc, frame_id)
          add_list.append(yb)
          o.add_new_layer(add_list)
          o.process_layer(len(o.layers) - 1)
          pafn.frame_draw_dot(screen, p, pafn.colors['green'])
          pygame.display.update()
          frame_id += 1

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  tracker_testing(screen)
  # moving_target(screen, [A, B], [T, T2])

if __name__ =='__main__':
  main()
  