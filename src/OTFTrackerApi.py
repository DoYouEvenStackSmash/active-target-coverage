
import numpy as np

from YoloBox import YoloBox
'''
Input format
  [
    {
      "id": 0,
      "image_id": 0,
      "category_id": 0,
      "bbox": [
        2270.90319824,
        1507.600341795,
        1423.2346191399997,
        607.2868652300001
      ],
      "area": 864311.6903443435,
      "segmentation": [],
      "iscrowd": 0,
      "track_id": 1,
      "trackmap_index": 0,
      "vid_id": 0,
      "track_color": [
        0,
        100,
        0
      ]
    }
  ]
'''

class StreamingAnnotations:
  def register_new_yolo_annotations(s, valid_frame_name="frame_", w_factor=None, h_factor=None):
    '''
    Load yolo format bounding box from a list of annotations in YOLO format
      class center_x  center_y  width height
    Wrapper for register_annotation
    Returns a list of YoloBoxes
    '''
    scale = lambda x,y,w,h: (x * w, y * h) # lambda for scaling an x,y coord
    yoloboxes = []

    for i in range(len(s)):
      if len(s[i]) < 10:
        continue
      b = s[i].split()
      cbx = [float(val) for val in b]
      cbx[1] = cbx[1] * w_factor
      cbx[2] = cbx[2] * h_factor
      cbx[3] = cbx[3] * w_factor
      cbx[4] = cbx[4] * h_factor

      yb = StreamingAnnotations.register_annotation(cbx[0], cbx[1:], f"{valid_frame_name}{len(yoloboxes)}")
      yoloboxes.append(yb)

    return yoloboxes

  def register_new_LOCO_annotations(annotation_arr):
    ''' 
    load yolo format bounding boxes from a list of annotations in LOCO format
    Register new detections in-situ, for building and interacting with tracks step by step
    Wrapper for register_annotation
    '''
    scale = lambda x,y,w,h: (x * w, y * h) # lambda for scaling an x,y coord
    yoloboxes = []
    cbx = [0] * 5
    for anno in annotation_arr:
      valid_frame_name = anno["image_id"]
      cbx[0] = anno["category_id"]
      bbox = anno["bbox"]
      cbx[1] = bbox[0]
      cbx[2] = bbox[1]
      cbx[3] = bbox[2]
      cbx[4] = bbox[3]
      
      yb = StreamingAnnotations.register_annotation(cbx[0], cbx[1:], valid_frame_name)
      yoloboxes.append(yb)
    
    return yoloboxes


  def register_annotation(class_id = 0, bbox = [], valid_frame_name = "frame_"):
    '''
    Registers an annotation as a YoloBox
    '''
    yb = YoloBox(class_id,bbox, valid_frame_name)
    return yb

