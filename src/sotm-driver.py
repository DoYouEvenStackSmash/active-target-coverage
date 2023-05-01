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

def get_tracks(steps):
  trail = []
  for s in steps:
    x,y,w,h = s['bbox']
    trail.append((x,y))
  return trail


def visible_targets(A, Tlist):
  pairs = []
  sortkey = lambda x: x[2]
  frame_id = "frame_"+str(len(A.obj_tracker.layers))
  for target in Tlist:
    d = mfn.euclidean_dist(A.origin, target.get_origin())
    pairs.append((A, target, d))
  pairs = sorted(pairs, key=sortkey)
  c = 0
  pl = []
  add_list = []
  while c < len(pairs):
    if pairs[c][2] > pairs[c][0].fov_radius:
      break
    if pairs[c][0].is_visible(pairs[c][1].get_origin()):
      dc = A.compute_detection_coords(pairs[c][1].get_origin())
      # x,y = pairs[c][1].get_origin()
      # dc = [x,y,1,1]

      yb = sann.register_annotation(0, dc, frame_id)
      add_list.append(yb)
    c+=1
  A.obj_tracker.add_new_layer(add_list)
  A.obj_tracker.process_layer(len(A.obj_tracker.layers) - 1)

def export_tracks(A, screen):
  print(A.obj_tracker.global_track_store)
  # steps = []
  # A.obj_tracker.process_all_layers()
  A.obj_tracker.close_all_tracks()
  
  A.obj_tracker.link_all_tracks(0)
  tracks = A.obj_tracker.linked_tracks
  print(len(A.obj_tracker.layers))
  # print(tracks)
  track_steps = []
  for i in tracks:
    steps = []
    A.obj_tracker.get_track(i).get_loco_track(fdict=None, steps=steps)
    track_steps.append(steps)
  for s in track_steps:
    for st in range(len(s)):
      bbox = s[st]["bbox"]
      x,y = A.decompute_detection_coords(bbox)
      s[st]["bbox"][0],s[st]["bbox"][1] = x,y
      # x,y = (bbox[0] / 1920) * A.fov_theta - A.fov_width / 2, bbox[1]
      # print(st)
    print("-----------------------------")
    t = get_tracks(s)
    for i in range(1, len(t)):
      pafn.frame_draw_line(screen, (t[i-1], t[i]), pafn.colors["magenta"])
  pgn = A.get_polygon()
  pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
    # print(s)
  

def moving_target(screen, Alist, Tlist):
  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        if pygame.key.get_mods() == LCTRL:
          pafn.clear_frame(screen)
          for A in Alist:
            # pgn = A.get_polygon()
            # pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
            export_tracks(A, screen)
          pygame.display.update()

          # sys.exit()
        elif pygame.key.get_mods() == LALT:
          pafn.clear_frame(screen)
          for A in Alist:
            pgn = A.get_polygon()
            pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
          pygame.display.update()
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          translation_path = []
          translation_path = gfn.lerp_list(Tlist[0].get_origin(), p, 8)
          # for A in Alist:
          # pgn = A.get_polygon()
          for pt in translation_path:
            # pafn.clear_frame(screen)
            theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
            for t in range(len(Tlist)):
              Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
            
            pafn.clear_frame(screen)
            # T.origin = pt
            for a in range(len(Alist)):
              A = Alist[a]
              visible_targets(A, Tlist)
              pgn = A.get_polygon()
              pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
              # A.rotate(Tlist[0].get_origin())
            # pv = visibility(A, [T])
            # add_visible_to_tracker(pv)
              if A.obj_tracker.active_tracks != None and len(A.obj_tracker.active_tracks):
                colors = list(set(pafn.colors))
                c = 0
                for trk in A.obj_tracker.active_tracks:
                  # trk = A.obj_tracker.active_tracks[0]
                  # print(colors[c])
                  pred_pt = trk.predict_next_box()
                  x,y = pred_pt
                  bbox = [x,y,1,1]
                  pred_pt = A.decompute_detection_coords(bbox)
                  pafn.frame_draw_dot(screen, pred_pt, pafn.colors["red"])
                  A.rotate(pred_pt)
                  c+=1
              # pafn.clear_frame(screen)
              pgn = A.get_polygon()
              pafn.frame_draw_polygon(screen, pgn, pafn.colors['yellow'])
            
            # for yb in lv:
            #   print(yb.bbox)
            for t in range(len(Tlist)):
              pt = Tlist[t].get_origin()
              pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
            # pafn.frame_draw_dot(screen, pt, pafn.colors["green"])
            # for p in pv:
            #   pafn.frame_draw_dot(screen, p[1].get_origin(), pafn.colors["magenta"])
            # pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
            pygame.display.update()
            time.sleep(0.01)

detections = [{
  "image_id": "frame_1",
  "category_id": 0,
  "bbox" : [0.0,
            1.0,
            1.0,
            1.0]
  }]


# def main():
#   OTM = ObjectTrackManager()
#   OTM.init_new_layer()
#   layer = sann.register_new_LOCO_annotations(detections)
#   print(layer)
#   OTM.add_new_layer(layer)
#   print(OTM.get_layer(1))

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  layer = sann.register_new_LOCO_annotations(detections)
  A = Agent([400,400], [np.pi/4, 300, np.pi / 8], obj_tracker = ObjectTrackManager())
  # A.color = pafn.
  B = Agent([600,600], [np.pi/8, 100, np.pi / 8], obj_tracker = ObjectTrackManager())
  T = Target((500,500))
  T2 = Target((600,600))
  
  # print(A.obj_tracker.get_layer())
  # moving_target(screen, [A], T)
  moving_target(screen, [A], [T])#,T2])

if __name__ =='__main__':
  main()
  