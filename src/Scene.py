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
from Target import Target

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
  '''
  Determines visible targets in the vicinity of agent A and updates
  the agent A's tracker.

  Does not return
  '''
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
      dc = A.transform_to_local_coord(pairs[c][1].get_origin())

      yb = sann.register_annotation(0, dc, frame_id)
      add_list.append(yb)
    c+=1
  A.obj_tracker.add_new_layer(add_list)
  A.obj_tracker.process_layer(len(A.obj_tracker.layers) - 1)


def draw_coordinate_frame(screen, A):
  '''
  Helper function for displaying the curved coordinate fov of agent A

  Does not return
  '''
  coord_frame = A.get_sensor_field(5)
  for level in coord_frame:
    for i in range(1, len(level)):
      pafn.frame_draw_line(screen, (level[i-1], level[i]), pafn.colors['white'])
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (A.origin, endpoint), pafn.colors['white'])


def import_loco_fmt(s, sys_path):
  '''
  Imports a LOCO formatted json

  Returns a json of some sort
  '''

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