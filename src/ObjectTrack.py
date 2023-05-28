#!/usr/bin/python3
import numpy as np
from aux_functions import *
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
import sys


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


class ObjectTrack:
    """A class representing an object track.

    Attributes:
        r (float): Distance between the two most recent elements in the track.
        theta (List[float]): Angle between the two most recent elements in the track.
        delta_theta (List[float]): Change in angle after adding an element to the track.
        delta_v (List[float]): Change in velocity (acceleration) after adding an element to the track.
        v (List[float]): Velocity between every adjacent pair of elements in the track.
        path (List[YoloBox]): List of all detections in the track.
        predictions (List[List[YoloBox]]): List of predictions for the track.
        track_id (int): Unique identifier for the track.
        color (str): Color for visualizing the object track.
        last_frame (int): Frame ID of the latest detection.
        class_id (int): Track classification for use with an Object Detector.
        error_over_time (List[float]): Accumulation list for absolute error.
        detection_idx (List[int]): Time stamps of detections.
        detection_time (int): Total time running detections.
        avg_detection_time (int): Average time between detections.
        clock (int): Counter.
    """

    def __init__(self, track_id, class_id):
        """Initializes an ObjectTrack instance.

        Args:
            track_id (int): Unique identifier for the track.
            class_id (int): Track classification for use with an Object Detector.
        """
        self.r = 0
        self.theta = [0]
        self.delta_theta = [0]
        self.delta_v = [0]
        self.v = [0]
        self.path = []
        self.predictions = [[]]
        self.track_id = track_id
        self.color = rand_color()
        self.last_frame = -1
        self.class_id = class_id
        self.error_over_time = []
        self.detection_idx = []
        self.detection_time = 1
        self.avg_detection_time = 1
        self.clock = 0
        self.parent_agent = None

    def heartbeat(self):
        """
        Heartbeat function
        """
        self.clock += 1

    def add_new_detection(self, detection, frame_id, error=-1):
        """
        Add a new bounding box to the object track
        """
        self.last_frame = frame_id
        detection.parent_track = self.track_id
        self.path.append(detection)

    def add_new_prediction(self, pred):
        """
        Wrapper for object track adding its own prediction
        """
        pass

    def update_track_trajectory(self, pt, displacement=None):
        """
        Update trajectory with a new measurement
        Assumes path is not empty
        """
        pass

    def estimate_next_position(self):
        """
        Estimate position of next detection using absolute measurements
        """
        pass

    def predict_next_state(self, steps=1):
        """
        Predict an intermediate position of the target using its
        movement characteristics, scaled by the average detection time
        """
        pass

    def get_state_estimation(self):
        """
        Using filters, estimate the future kinematic state of the target
        using a combination of measurements and predictions
        """
        if len(self.path):
            return self.path[-1]
        return None
        # pass

    def get_track_heading(self):
        """
        Accessor for track trajectory information
        Returns the track heading
        """
        return (self.get_last_detection(), self.r, self.delta_v[-1], self.theta[-1])

    def is_alive(self, fc, expiration):
        """
        Check whether a track is expired
        Args:
            fc (int): current frame counter from ObjectTrackManager
            Expiration (int): lifetime of the track
        """
        return bool(fc - self.last_frame < expiration)

    def link_path(self):
        """
        Postprocessing step to construct a doubly linked list, in preparation for serialization
        """
        for i in range(len(self.path) - 1):
            self.path[i].next = self.path[i + 1]
            self.path[i + 1].prev = self.path[i]

    def get_step_count(self):
        """
        Accessor for checking length of the track
        """
        return len(self.path)

    def get_last_detection(self):
        """
        Accessor for getting the last detection in a track
        returns a Detection
        """
        if len(self.path) > 0:
            return self.path[-1]
        return None
    
    def get_last_detection_coordinate(self):
        """
        Accessor for the coordinate of the last detection
        """
        if len(self.path) > 0:
            return self.path[-1].get_center_coord()
        return None

    def get_loco_track(self, fdict=None, steps=None):
        """
        Get complete track in loco format
        template = {
                    "id":counter,
                    "image_id": 0,
                    "category_id":1.0,
                    "bbox" : [
                        cx,
                        cy,
                        w,
                        h
                    ],
                    "segmentation": [],
                    "iscrowd": 0,
                    "track_id" : self.track_id,
                    "vid_id":0
                    }
        """
        steps = steps if steps != None else []

        for i, det in enumerate(self.path):
            yb = det.get_attributes()
            fid = None
            # if fdict != None:
            #   fid = fdict[f'{yb.img_filename[:-3]}png']
            fid = yb.img_filename
            steps.append(
                yb.to_json(fid, self.error_over_time[i], fid, self.color)
            )
