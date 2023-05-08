#!/usr/bin/python3
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
#from Agent import *

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


def drawing(screen):
  s = json_loader("out.json")
  annos = import_loco_fmt(s,None)
  scaling = 5
  boundary_offt = 50 * scaling
  # crosshair_offt = 15 * scaling
  origin = (400,400)
  crosshairs = [[(385,400), (415,400)], [(400,385), (400, 415)]]
  b_min = 400 - boundary_offt
  b_max = 400 + boundary_offt
  step = (b_max - b_min)/ 4

  boundary = [(b_min,b_min), (b_min,b_max), (b_max,b_max), (b_max,b_min)]
  axes = []#[(b_min,400)]
  for i in range(2):
    axes.append((b_min + step * i, 400))
  axes.append((400,400))
  
  axes.append((b_max - step * 1, 400))
  axes.append((b_max, 400))
  # for i in range(2):
  start_pt = ((b_max - b_min) / 2 + b_min, b_max)

  for pt in axes:
    pafn.frame_draw_line(screen, (start_pt, pt), pafn.colors['cyan'])
  lh_theta, lh_r = mfn.car2pol(start_pt, axes[0])
  rh_theta, rh_r = mfn.car2pol(start_pt,axes[-1])
  arr = [1,2,4,8,16,32]
  # arr.reverse()
  step = lh_r / 32
  pts = []
  for a in arr:
    pts.append((mfn.pol2car((b_min,400), -1 * a * step,lh_theta),mfn.pol2car((b_max,400), -1 * a * step,rh_theta)))
  for p in pts:
    pafn.frame_draw_line(screen, p, pafn.colors['cyan'])
  pygame.display.update()
  # time.sleep(3)
  # boundary = [(400,step + 400), (,b_max), (b_max,b_max), (b_max,b_min)]

  center = 400
  disp = 0
  center = annos[0]["bbox"][0]
  for a in annos:
    x,y = a["bbox"][0], 400
    x = (x - 50) * scaling + 400
    # if a['displaced']:
    #   disp = x - center
    #   center = x
      # center = x
      # x = center % 400 
    x = x + disp
    pafn.clear_frame(screen)
    pafn.frame_draw_polygon(screen, boundary, pafn.colors['indigo'])
    for pt in axes:
      pafn.frame_draw_line(screen, (start_pt, pt), pafn.colors['cyan'])
    for p in pts:
      pafn.frame_draw_line(screen, p, pafn.colors['cyan'])

    for c in crosshairs:
      pafn.frame_draw_line(screen, c, pafn.colors['magenta'])
    pafn.frame_draw_dot(screen, (x,y), a["track_color"])
    pygame.display.update()
    time.sleep(0.2)
  

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  drawing(screen)

if __name__ == '__main__':
  main()