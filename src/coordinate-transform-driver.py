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

def moving_target(screen, A):
  dest = None
  bbox = None
  orig = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        if pygame.key.get_mods() == LCTRL:
          sys.exit()

        elif pygame.key.get_mods() == LALT:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pafn.frame_draw_dot(screen, p, pafn.colors["red"])
          orig = p
          bbox = A.transform_to_local_coord(p)
        elif pygame.key.get_mods() == LSHIFT:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          new_p = A.transform_from_local_coord(bbox)
          pafn.frame_draw_dot(screen, new_p, pafn.colors["green"])
          print(f"orig: {orig}\nbbox: {new_p}")
        # else:
      pgn = A.get_polygon()
      pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
      pygame.display.update()

        
def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  
  A = Agent([400,400], [np.pi/4, 300, np.pi / 4], obj_tracker = ObjectTrackManager())
  # A.color = pafn.
  B = Agent([600,600], [np.pi/8, 100, np.pi / 8], obj_tracker = ObjectTrackManager())
  T = Target((500,500))
  T2 = Target((600,600))
  
  # print(A.obj_tracker.get_layer())
  # moving_target(screen, [A], T)
  moving_target(screen, A)

if __name__ =='__main__':
  main()
  