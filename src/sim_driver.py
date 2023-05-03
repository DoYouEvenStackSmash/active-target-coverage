#!/usr/bin/python3

from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from Agent import *

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

def determine_visible(As, pts):
  pairs = []
  sortkey = lambda x: x[2]
  
  for i in As:
    # i.pose_adjustments.append(None)
    for j in pts:
      d = mfn.euclidean_dist(i.origin, j)
      pairs.append((i, j, d))
  pairs = sorted(pairs, key=sortkey)
  c = 0
  pl = []
  while c < len(pairs):
    if pairs[c][2] > pairs[c][0].fov_radius:
      break
    if pairs[c][0].is_visible(pairs[c][1]):
      print(f"{pairs[c][0]._id} has f{pairs[c][1]}")
      pl.append((pairs[c][0].origin, pairs[c][1]))
    c+=1
  return pl

def center_track(screen, A):
  
  origin = (500,500)
  pgn = A.get_polygon()
  print(pgn)
  pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
  pafn.frame_draw_dot(screen, A.origin, pafn.colors["green"],10)
  pygame.display.update()
  ptlist = []
  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        if pygame.key.get_mods() == LCTRL:
          sys.exit()
        elif pygame.key.get_mods() == LALT:
          A.rotate(pygame.mouse.get_pos())
          pafn.clear_frame(screen)
          pgn = A.get_polygon()
          print(pgn)
          pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
          for i in pgn:
            pafn.frame_draw_dot(screen, i, pafn.colors["green"],10)
          pygame.display.update()
          continue
        elif pygame.key.get_mods() == LSHIFT:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          dest = p
          print(f"set destination!")
          continue
        
        while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
          continue
        p = pygame.mouse.get_pos()
        ptlist.append(p)
        pafn.frame_draw_dot(screen, p, pafn.colors["green"],10)
        if len(ptlist) < 5:
          pygame.display.update()
          continue
        translation_path = []
        if dest != None:
          translation_path = gfn.lerp_list(A.origin, dest, 30)
        
        pairlist = []
        for pt in translation_path:
          A.origin = pt
          pl = determine_visible([A], ptlist)
          for p in pl:
            pairlist.append(pl)
        
        # pafn.clear_frame(screen)
        # print(pl)
        for ptl in range(len(translation_path)):
          # pafn.clear_frame(screen)
          A.origin = translation_path[ptl]
          # pgn = A.get_polygon()
          # pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
          if ptl < len(pairlist):
            pl = pairlist[ptl]
            for pair in pl:
              pafn.frame_draw_bold_line(screen, pair, pafn.colors['magenta'])
          pygame.display.update()
          time.sleep(0.01)
          
        ptlist = []
        dest = None




          # sys.exit()
        # construct the path
        # while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
        #   continue

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  A = Agent([400,400], [np.pi/10, 100, np.pi / 8])
  center_track(screen, A)

if __name__ == '__main__':
  main()