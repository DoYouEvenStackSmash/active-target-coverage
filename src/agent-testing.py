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

def agent_testing(screen, A):
  frame_id = 0
  o = ObjectTrackManager()
  pgn = A.get_polygon()
  pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
  coord_frame = A.get_sensor_field(5)
  for level in coord_frame:
    for i in range(1, len(level)):
      pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])

  pygame.display.update()
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
          if A.is_visible(p):
            dc = A.transform_to_local_coord(p)
            print(dc)
            # x,y = dc
            rc = A.transform_from_local_coord(dc)
            print(f"p:{p}\trc:{rc}")

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  A = Agent([500,500], [0, 400, np.pi / 4], obj_tracker = ObjectTrackManager())
  pygame.display.update()
  agent_testing(screen,A)

if __name__ =='__main__':
  main()
