
import numpy as np
from YoloBox import YoloBox
from aux_functions import ImgFxns
from os import path
import json

class AnnotationLoader:
  const_w = 1920
  const_h = 1080
  def load_annotation_file_list(valid_file, sys_path = "."): 
    '''
    Load filenames from a file for further preprocessing
    Returns a list of filenames(strings)
    '''
    if not path.exists(valid_file):
      print(f"{valid_file} does not exist!")
      return None
    
    f = open(f"{valid_file}","r")
    s = f.readlines()
    f.close()
    s = [i.rstrip("\n") for i in s]
    return s
  
  def load_annotations_from_json_file(valid_file):
    '''
    Placeholder to load annotations from a valid json file
    Returns a python dictionary object
    '''
    if not path.exists(valid_file):
      print(f"{valid_file} does not exist!")
      return {}
    
    # generic file opening
    f = open(valid_file,"r")
    s = f.readlines()
    f.close()
    
    # json string to python dictionary
    s = "".join([i.rstrip("\n") for i in s])
    s = json.loads(s)
    return s
    
  
  ''' YOLO CONSTANTS '''
  YOLOX_LEN = 6 # class confidence min_x min_y max_x max_y
  YOLO_LEN = 5  # class center_x center_y width height
  

  def load_annotations_from_text_file(valid_file):
    '''
      Read and split lines from a valid txt annotation file.
      Returns a list of strings
    '''
    if not path.exists(valid_file):
      print(f"{valid_file} does not exist!")
    
    f = open(f"{valid_file}","r")
    s = f.readlines()
    f.close()
    s = [i.rstrip("\n") for i in s]    
    return s
  
  def load_yolofmt_layer(valid_png_file):
    '''
    Load annotations from yolo/x formatted files
    Returns a (possibly empty) list of yoloboxes
    '''
    # expects *.png or similar
    image_w, image_h = ImgFxns.get_img_shape(valid_png_file)
    valid_filename = valid_png_file[:-3] + "txt"

    annotations = AnnotationLoader.load_annotations_from_text_file(valid_filename)
    if len(annotations) == 0:
      print(f"EMPTY FILE: {valid_filename}")
      return []
    
    # select yolo parser
    if len(annotations[0].split()) == AnnotationLoader.YOLOX_LEN:
      return AnnotationLoader.parse_yolox_annotations(annotations, valid_filename)
    elif len(annotations[0].split()) == AnnotationLoader.YOLO_LEN:
      return AnnotationLoader.parse_yolo_annotations(annotations, valid_filename, image_w, image_h)
    else:
      print(f"SKIPPING {valid_filename}: ANNOTATIONS FORMAT NOT RECOGNIZED")
      return []

  def parse_yolox_annotations(s, valid_file):
    '''
    Load yolox bounding box data of a specific frame from a valid file
    Returns a list of YoloBoxes
    
    yolovx bbox format
    (0,0)        (max_w,0)
      +---------------+
      |               |
      |               |
      |               |
      +---------------+
    (0,max_h)    (max_w,max_h)

    Yolox Bboxes as output from the yolox onnx_inference:
      class  confidence  min_x min_y max_x max_y
    '''
    # process each annotation, 1 per line
    yoloboxes = []
    for i in range(len(s)):
      
      b = s[i].split()
      bx = [float(val) for val in b[2:]]
      cbx = YoloBox.conv_yolox_bbox(bx)
      yoloboxes.append(YoloBox(float(b[0]), cbx, valid_file, float(b[1])))
    
    return yoloboxes
  
  def parse_yolo_annotations(s, valid_file, w_factor=None, h_factor=None):
    '''
    Load yolo format bounding box from text file
      class center_x  center_y  width height
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
      yoloboxes.append(YoloBox(cbx[0],cbx[1:], valid_file))

    return yoloboxes


