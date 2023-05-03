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
import json
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
  # A.pose_adjustments.append(None)
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

def export_linked_loco_tracks(A,fdict = None):
  '''
    build "annotations" : [] from linked tracks only
  '''
  OTM = A.obj_tracker
  track_steps = []
  for i in OTM.linked_tracks:
    # steps = []
    OTM.get_track(i).get_loco_track(fdict=None,steps=track_steps)
  for st in range(len(track_steps)):
    bbox = track_steps[st]["bbox"]
    x,y,w,h = bbox
    # x,y = A.transform_from_local_coord(x,y,w,h)
    bbox[0],bbox[1] = x,y
    track_steps[st]["bbox"] = bbox
  # print(f'{len(steps)} total steps')
  for i in range(len(track_steps)):
    track_steps[i]["id"] = i
  return track_steps

def export_loco_fmt(A):
  '''
  Export active tracks and associated metadata to loco format
  '''
  # construct filename lookup dictionary
  fdict = None
  # construct "images" : []
  imgs = []
  OTM = A.obj_tracker
  # construct "annotations" : []
  steps = export_linked_loco_tracks(A,fdict)
  
  '''
  Generate new images with which to populate a LOCO of the REFLECTED images
  '''

  # construct "linked_tracks" : []
  linked_tracks = [{"track_id": i, "category_id" : OTM.get_track(i).class_id, 
                    "track_len": 0, "steps":[] } 
                    for i in OTM.linked_tracks]

  
  trackmap = {} # {track_id : posn in linked_tracks}
  for i,lt in enumerate(linked_tracks):
    trackmap[linked_tracks[i]['track_id']] = i
  
  # add trackmap_index to all annotations
  for s in steps:
    linked_tracks[trackmap[s['track_id']]]['steps'].append(s['id'])
    s['trackmap_index'] = trackmap[s['track_id']]
  
  # add length to linked tracks for fun
  for l in linked_tracks:
    l["track_len"] = len(l['steps'])
  
  # assemble final dictionary
  exp = {
          "constants": ObjectTrackManager.constants,
          "categories":OTM.categories,
          "trackmap":list(trackmap),
          "linked_tracks":linked_tracks,
          "images":imgs, 
          "annotations":steps
        }
  return exp

def export_tracks(screen, A):
  # print(A.obj_tracker.global_track_store)
  # steps = []
  # A.obj_tracker.process_all_layers()
  A.obj_tracker.close_all_tracks()
  A.obj_tracker.link_all_tracks(0)
  e = export_loco_fmt(A)
  f = open("out.json", "w")
  f.write(json.dumps(e, indent = 2))
  f.close()

  # tracks = A.obj_tracker.linked_tracks
  # # print(len(A.obj_tracker.layers))
  # # print(tracks)
  # track_steps = []

  # for i in tracks:
  #   steps = []
  #   A.obj_tracker.get_track(i).get_loco_track(fdict=None, steps=steps)
  #   track_steps.append(steps)
  # for s in track_steps:
  #   json.dumps(s,indent=2)
  #   for st in range(len(s)):
  #     bbox = s[st]["bbox"]
  #     x,y,w,h = bbox
  #     x,y = A.transform_from_local_coord(x,y,w,h)
  #     bbox[0],bbox[1] = x,y
  #     s[st]["bbox"] = bbox
  #   #   s[st]["bbox"][0],s[st]["bbox"][1] = x,y
  #   #   # x,y = (bbox[0] / 1920) * A.fov_theta - A.fov_width / 2, bbox[1]
  #   #   print(st)
  #   # print("-----------------------------")
  #   print(json.dumps(s,indent=2))
  #   t = get_tracks(s)
  #   for i in range(1, len(t)):
  #     pafn.frame_draw_line(screen, (t[i-1], t[i]), pafn.colors["magenta"])
  # pgn = A.get_polygon()
  # pafn.frame_draw_polygon(screen, pgn, pafn.colors['tangerine'])

def draw_coordinate_frame(screen, A):
  coord_frame = A.get_sensor_field(5)
  for level in coord_frame:
    for i in range(1, len(level)):
      pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])


def import_loco_fmt(s, sys_path):
  # set up trackmap for accessing tracks
  # self.imported = True
  # trackmap = s['trackmap']
  # lt = s['linked_tracks']
  # for i,track_id in enumerate(trackmap):
  #   if track_id not in self.global_track_store or track_id == -1:
  #     self.global_track_store[track_id] = ObjectTrack(track_id, lt[i]['category_id'])
  #     self.global_track_store[track_id].class_id = lt[i]['category_id']
  #   else: #already present
  #     continue
  
  # load image filenames
  # images = s['images']
  # # construct file dict for accessing file ids
  # # construct sys_paths list for convenience
  # # initialize layers to populate with YoloBoxes
  # for i,imf in enumerate(images):
  #   self.filenames.append(imf['file_name'])
  #   self.sys_paths.append(sys_path)
  #   self.fdict[imf['file_name']] = i
  #   self.layers.append([])
  #   self.img_centers.append(tuple((int(imf['width']/2), int(imf['height']/2))))
  
  # load annotations
  steps = s['annotations']
  # for st in steps:
  #   # skip step if track is invalid
  #   if trackmap[st['trackmap_index']] == -1:
  #     continue
  #   track = self.get_track(trackmap[st['trackmap_index']])
  #   yb = YoloBox( track.class_id, 
  #                 st['bbox'], 
  #                 f'{self.filenames[st["image_id"]][:-3]}txt',
  #                 self.img_centers[st["image_id"]])
    
  #   # add YoloBox to the appropriate layer based on the image filename
  #   self.layers[self.fdict[self.filenames[st['image_id']]]].append(yb)
  #   # add the yolobox to the correct track
  #   track.add_new_step(yb,0)
  # bboxes = []

  return steps

def json_loader(filename):
  '''
  LOADER
  legacy json loader
  Takes a filename and sys_path, opens a json file
  
  Returns a python dict
  '''
  an_json = al.load_annotations_from_json_file(filename)
  if len(an_json) == 0:
    print("failed to load tracks")
    return None
  return an_json