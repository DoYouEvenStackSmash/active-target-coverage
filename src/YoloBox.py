#!/usr/bin/python3
from aux_functions import *
import numpy as np

'''
  Generic class for yolo annotation data
'''
class YoloBox:
  '''
    class_id  : index of class in obj.data
        bbox  : bounding box [centerx, centery, width, height]. assumes uniform dataset
    img_file  : identifier for mapping bounding box to source image
    confidence: optional confidence value from inference
  '''
  def __init__(self,class_id, bbox, img_filename, center_xy = None, confidence = None, distance = None):
    self.class_id = class_id
    self.bbox = bbox
    self.img_filename = img_filename
    self.confidence = confidence
    self.parent_track = None
    self.next = None
    self.prev = None
    self.center_xy = center_xy
    self.distance = distance


  def get_corner_coords(self):
    '''
    Helper for translating yolo bbox to [(minx, miny), (maxx, maxy)]
    Returns a list of points
    '''
    center_x, center_y, w, h = self.bbox
    minx,miny,maxx,maxy = center_x - w/2, center_y - h/2, center_x + w/2, center_y + h/2
    return [(minx, miny), (maxx, maxy)]
  
  def get_center_coord(self):
    '''
    Accessor for center of bounding box
      returns (x, y)
    '''
    return (self.bbox[0],self.bbox[1])

  
  def conv_yolox_bbox(bbox):
    '''
    Convert yolox bounding box to yolo format
    yolox :  [minx, miny, maxx, maxy]
    yolo  :  [centerx, centery, width, height]
    Returns a yolo bounding box
    '''
    pt1,pt2 = bbox[:2],bbox[2:]
    # find midpoint via polar coordinates
    r = MathFxns.euclidean_dist(pt1,pt2)
    theta = np.arctan2(pt2[0] - pt1[0], pt2[1] - pt1[1])
    mpx,mpy = r/2 * np.sin(theta) + pt1[0], r/2 * np.cos(theta) + pt1[1]
    
    # calculate width and height
    w,h = abs(pt1[0] - pt2[0]),abs(pt1[1] - pt2[1])
    return [mpx, mpy, w, h]
  
  def rotate_quad(self,dir_flag):
    '''
    Rotate a yolobox 90 degrees
    Since the expectation is a transpose of the image, center is changed
    '''
    cbx,cby,w,h = self.bbox
    cx,cy = self.center_xy
    self.bbox[0] = cy + ((cby - cy) * dir_flag)
    self.bbox[1] = cx + ((cbx - cx) * dir_flag * -1)
    self.bbox[2] = h
    self.bbox[3] = w
    
    # swap dimensions after rotation
    self.center_xy = cy,cx
  
  def reflectXY(self):
    '''
    Reflect a box across the x, then the Y axis. 
    Equivalent to a rotation by 180 degrees
    '''
    # cbx,cby,w,h = self.bbox
    cx,cy = self.center_xy
    self.bbox[0] = cx + ((self.bbox[0] - cx) * -1)
    self.bbox[1] = cy + ((self.bbox[1] - cy) * -1)
  
  def reflectX(self):
    '''
    reflect a box across the y axis 
    '''
    cx = self.center_xy[0]
    self.bbox[0] = cx + ((self.bbox[0] - cx) * -1)

  def reflectY(self):
    '''
    reflect a box across the X axis
    '''
    cy = self.center_xy[1]
    self.bbox[1] = cy + ((self.bbox[1] - cy) * -1)
