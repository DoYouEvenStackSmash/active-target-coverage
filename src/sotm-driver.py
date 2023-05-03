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
  A.pose_adjustments.append(None)
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
      dc = A.transform_to_local_coord(pairs[c][1].get_origin())

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
      x,y = A.transform_from_local_coord(bbox)
      s[st]["bbox"][0],s[st]["bbox"][1] = x,y
      # x,y = (bbox[0] / 1920) * A.fov_theta - A.fov_width / 2, bbox[1]
      print(st)
    print("-----------------------------")
    t = get_tracks(s)
    for i in range(1, len(t)):
      pafn.frame_draw_line(screen, (t[i-1], t[i]), pafn.colors["magenta"])
  pgn = A.get_polygon()
  pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
    # print(s)
  
def estimate_next_detection(otm):
  curr_pt, pred_pt = (),()
  if otm.active_tracks != None and len(otm.active_tracks):
    print(f"active tracks: {len(otm.active_tracks)}")
    trk = otm.active_tracks[0]
    pred_pt = trk.predict_next_box()
    curr_pt = trk.get_last_detection()
  return (curr_pt, pred_pt)

def moving_target(screen, Alist, Tlist):
  last_posn = (0,0)
  for A in Alist:
    coord_frame = A.get_sensor_field(5)
    for level in coord_frame:
      for i in range(1, len(level)):
        pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
    for endpoint in coord_frame[-1]:
      pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])
    # pgn = A.get_polygon()
    # pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])
  for t in range(len(Tlist)):
    ptr = Tlist[t].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  pygame.display.update()

  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        if pygame.key.get_mods() == LCTRL:  # rotate
          pafn.clear_frame(screen)
          for A in Alist:
            A.rotate(pygame.mouse.get_pos())
            coord_frame = A.get_sensor_field(5)
            for level in coord_frame:
              for i in range(1, len(level)):
                pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
            for endpoint in coord_frame[-1]:
              pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])
            pafn.frame_draw_dot(screen, last_posn, pafn.colors["green"])
            
            # export_tracks(A, screen)
          pygame.display.update()
          # sys.exit()
        elif pygame.key.get_mods() == LALT: # estimate
          pafn.clear_frame(screen)
          print(f"{'-'*100}")
          for A in Alist:
            A.targets_covered()
          print(f"{'='*100}")
          for a in range(len(Alist)):
            
            A = Alist[a]
            next_det = A.estimate_next_detection()
            curr, pred = next_det
            if len(pred) > 0:
              pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
              pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])
              # A.rotate(pred)
            if len(curr) > 0:
              pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
            pafn.frame_draw_dot(screen, last_posn, pafn.colors["green"])
            coord_frame = A.get_sensor_field(5)
            for level in coord_frame:
              for i in range(1, len(level)):
                pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
            for endpoint in coord_frame[-1]:
              pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])
            
          pygame.display.update()
        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          last_posn = p
          translation_path = []
          translation_path = gfn.lerp_list(Tlist[0].get_origin(), p, 4)
          print("running\n")

          for pt in translation_path[1:]:
            # pafn.clear_frame(screen)
            
            pafn.clear_frame(screen)
            # T.origin = pt
            for a in range(len(Alist)):
              A = Alist[a]
              next_det = A.estimate_next_detection()
              curr, pred = next_det
              if len(pred) > 0:
                pafn.frame_draw_dot(screen, pred, pafn.colors['yellow'])
                pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])
                A.rotate(pred)
              if len(curr) > 0:
                pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
              
              coord_frame = A.get_sensor_field(5)
              for level in coord_frame:
                for i in range(1, len(level)):
                  pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
              for endpoint in coord_frame[-1]:
                pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])
            time.sleep(.2)

            theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
            for t in range(len(Tlist)):
              Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
              ptr = Tlist[t].get_origin()
              pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
            
            for a in range(len(Alist)):
              visible_targets(Alist[a], Tlist)

            pygame.display.update()
            

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
  A = Agent([200,200], [0, 700, np.pi / 8], obj_tracker = ObjectTrackManager())
  # A.color = pafn.
  B = Agent([600,600], [np.pi/8, 200, np.pi / 6], obj_tracker = ObjectTrackManager())
  T = Target((500,500))
  T2 = Target((600,600))
  
  # print(A.obj_tracker.get_layer())
  moving_target(screen, [A], [T])
  # moving_target(screen, [A, B], [T, T2])

if __name__ =='__main__':
  main()
  