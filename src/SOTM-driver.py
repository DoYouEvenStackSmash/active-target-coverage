#!/usr/bin/python3
import numpy as np
from PIL import Image, ImageDraw
import collections
from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
import sys
import os
import json

detections = [{
  "image_id": "frame_1",
  "category_id": 0,
  "bbox" : [0.0,
            1.0,
            1.0,
            1.0]
  }]


def main():
  OTM = ObjectTrackManager()
  OTM.init_new_layer()
  layer = sann.register_new_LOCO_annotations(detections)
  print(layer)
  OTM.add_new_layer(layer)
  print(OTM.get_layer(1))

main()
