#!/usr/bin/python3
from aux_functions import *
import numpy as np

"""
  Generic class for yolo annotation data
"""

class YoloBox:
    """A class representing a single detection in Yolo format
    Attributes:
        class_id(int)  : index of class in obj.data
        bbox  (List[float]): bounding box [centerx, centery, width, height]. assumes uniform dataset
        img_file (string) : identifier for mapping bounding box to source image
        confidence (float): optional confidence value from inference
    """

    def __init__(
        self,
        class_id,
        bbox,
        img_filename,
        center_xy=None,
        confidence=None,
        distance=None,
    ):
        self.class_id = class_id
        self.bbox = bbox
        self.img_filename = img_filename
        self.confidence = confidence
        self.parent_track = None
        self.next = None
        self.prev = None
        self.center_xy = center_xy
        self.distance = distance
        self.displaced = False

    def get_corner_coords(self):
        """
        Helper for translating yolo bbox to [(minx, miny), (maxx, maxy)]
        Returns a list of points
        """
        center_x, center_y, w, h = self.bbox
        minx, miny, maxx, maxy = (
            center_x - w / 2,
            center_y - h / 2,
            center_x + w / 2,
            center_y + h / 2,
        )
        return [(minx, miny), (maxx, maxy)]

    def get_center_coord(self):
        """
        Accessor for center of bounding box
          returns (x, y)
        """
        return (self.bbox[0], self.bbox[1])

    def conv_yolox_bbox(bbox):
        """
        Convert yolox bounding box to yolo format
        yolox :  [minx, miny, maxx, maxy]
        yolo  :  [centerx, centery, width, height]
        Returns a yolo bounding box
        """
        pt1, pt2 = bbox[:2], bbox[2:]
        # find midpoint via polar coordinates
        r = MathFxns.euclidean_dist(pt1, pt2)
        theta = np.arctan2(pt2[0] - pt1[0], pt2[1] - pt1[1])
        mpx, mpy = r / 2 * np.sin(theta) + pt1[0], r / 2 * np.cos(theta) + pt1[1]

        # calculate width and height
        w, h = abs(pt1[0] - pt2[0]), abs(pt1[1] - pt2[1])
        return [mpx, mpy, w, h]
    
    def to_json(self, fid = None, error = -1 ,sid = -1, color = (255,255,255)):
        # print(self.bbox)
        return {
            "id": -1,
            "image_id": fid,
            "category_id": self.class_id,
            "bbox": self.bbox,
            "area": self.bbox[2] * self.bbox[3],
            "segmentation": [],
            "iscrowd": 0,
            "track_id": -1,
            "trackmap_index": -1,
            "vid_id": 0,
            "track_color": color,
            "displaced": self.displaced,
            "error": error,
            "state_id": sid,
        }
