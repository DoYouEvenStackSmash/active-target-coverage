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

    def heartbeat(self):
        """
        Heartbeat function
        """
        self.clock += 1

    def add_new_detection(self, yb, frame_id, error=-1):
        """
        Add a new bounding box to the object track
        """
        # update velocity
        self.heartbeat()

        self.detection_idx.append(self.clock)
        if len(self.path) > 0:
            self.detection_time = (
                self.detection_idx[-1] - self.detection_idx[-2] + self.detection_time
            )
            self.avg_detection_time = self.detection_time / len(self.detection_idx)

            self.update_track_vector(yb.get_center_coord())

        self.error_over_time.append(error)

        self.last_frame = frame_id
        yb.parent_track = self.track_id

        self.path.append(yb)
        self.predictions.append([])

    def add_new_prediction(self, pred):
        """
        Wrapper for object track adding its own prediction
        """
        if pred != None:
            self.predictions[-1].append(pred)
        self.heartbeat()
        return pred

    def update_track_vector(self, pt, displacement=None):
        """
        Update track trajectory information
        Assumes path is not empty
        """
        center = self.path[-1].get_center_coord()
        theta, distance = mfn.car2pol(center, pt)

        if len(self.v) > 1 and distance != 0:
            self.delta_v.append(min(1.1, distance / self.v[-1]))

        self.v.append(distance)

        self.r = distance

        self.theta.append(theta)

        # add recent velocity to delta_v

    def estimate_next_position(self):
        """
        Predict next bounding box center
        """
        # last_detection = self.detection_idx[-1]
        lx, ly = self.path[-1].get_center_coord()
        if len(self.path) == 1:
            return (lx, ly)

        last_distance = self.v[-1]
        distance = 0
        # consider acceleration in estimate
        last_change_in_distance = self.delta_v[-1]

        if last_change_in_distance != 0:
            distance = last_distance * last_change_in_distance
        else:
            distance = last_distance

        new_posn = mfn.pol2car((lx, ly), distance, self.theta[-1])

        return new_posn

    def predict_next_position(self):
        """
        Prediction for the next position by scaled theta and velocity, constant acceleration
        """
        # Need at least two detections to make a prediction
        if len(self.detection_idx) < 2:
            return None

        time_since_detection = max(1, self.clock - self.detection_idx[-1])
        scaling_factor = min(1, time_since_detection / self.avg_detection_time)


        lx, ly = self.path[-1].get_center_coord()

        if len(self.predictions[-1]) != 0:
            lx, ly, w, h = self.predictions[-1][-1].bbox

            time_since_detection = 1

        scaled_last_distance = self.v[-1] * scaling_factor
        distance = 0

        # consider acceleration in estimate
        last_change_in_distance = self.delta_v[-1]
        if last_change_in_distance > 1:
            last_change_in_distance = 1
        if last_change_in_distance != 0:
            distance = scaled_last_distance * last_change_in_distance
        else:
            distance = scaled_last_distance

        scaled_last_theta = self.theta[
            -1
        ] 

        predicted_posn = mfn.pol2car((lx, ly), distance, scaled_last_theta)
        return predicted_posn


    def predict_next_detection(self):
        """
        wrapper for estimating next bounding box center
        """
        estimated_detection = self.estimate_next_position()
        # self.add_new_prediction()
        predicted_posn = self.predict_next_position()
        # predicted_posn = None
        if self.clock - self.detection_idx[-1] > self.avg_detection_time * 10:
            return predicted_posn
        if predicted_posn != None:
            # estimated_detection = predicted_posn
            ed2 = gfn.get_midpoint(estimated_detection, predicted_posn)
            estimated_detection = gfn.get_midpoint(estimated_detection, ed2)

        return estimated_detection

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

    def reflect_track(self, reflect_axis=None):
        """
        DATA AUGMENTATION: Reflect across an axis
        """
        if reflect_axis == None:
            return
        for ybx in self.path:
            if reflect_axis == 1:
                ybx.reflectX()
            elif reflect_axis == 0:
                ybx.reflectY()
            else:
                break

    def rotate_track(self, offset_deg):
        """
        rotate all boxes on an object track
        Adjusts rotations to ensure rotation is always 90 degrees
          e.g. 270 -> -90
          180 -> reflectXY()
        """
        if offset_deg == 0:
            return

        dir_flag = 1
        if abs(offset_deg) == 180:
            for ybx in self.path:
                ybx.reflectXY()
            return

        # correct rotations to ensure 90 degrees
        if abs(offset_deg) == 270:
            offset_deg = 90 if offset_deg < 0 else -90

        # adjust direction
        if offset_deg < 0:
            dir_flag = -1

        # Rotate quadrant of each box
        for ybx in self.path:
            ybx.rotate_quad(dir_flag)

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
        """
        if len(self.path) > 0:
            return self.path[-1].get_center_coord()
        return ()

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
        
        for i, yb in enumerate(self.path):
            fid = None
            # if fdict != None:
            #   fid = fdict[f'{yb.img_filename[:-3]}png']
            fid = yb.img_filename
            steps.append(
                {
                    "id": -1,
                    "image_id": fid,
                    "category_id": yb.class_id,
                    "bbox": yb.bbox,
                    "area": yb.bbox[2] * yb.bbox[3],
                    "segmentation": [],
                    "iscrowd": 0,
                    "track_id": self.track_id,
                    "trackmap_index": -1,
                    "vid_id": 0,
                    "track_color": self.color,
                    "displaced": yb.displaced,
                    "error": self.error_over_time[i],
                    "state_id": fid,
                }
            )
