#!/usr/bin/python3
import numpy as np
from aux_functions import *
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn


class ObjectTrack:
    def __init__(self, track_id, class_id):
        self.r = 0  # distance between two most recent elements in the track
        self.theta = [0]  # angle between two most recent elements in the track
        self.delta_theta = [0]  # change in angle after adding element to the track
        self.delta_v = [
            0
        ]  # change in velocity (acceleration) after adding element to the track
        self.v = [0]  # velocity between every adjacent pair of elements in the track
        self.path = []  # list of all detections in the track
        self.track_id = track_id  # unique identifier for the track
        self.color = rand_color()
        self.last_frame = -1
        self.class_id = class_id  # track classification for use with an Object Detector

    def add_new_step(self, yb, frame_id):
        """
        Add a new bounding box to the object track
        """
        # update velocity
        if len(self.path) > 0:
            self.update_track_vector(yb.get_center_coord())

        self.last_frame = frame_id
        yb.parent_track = self.track_id
        self.path.append(yb)

    def update_track_vector(self, pt, displacement=None):
        """
        Update track trajectory information
        Assumes path is not empty
        """
        center = self.path[-1].get_center_coord()
        theta, r = mfn.car2pol(center, pt)
        if len(self.v) > 1 and r != 0:
          self.delta_v.append(min(1.1, (r / self.v[-1]) / self.delta_v[-1] ))
          print(self.delta_v[-1])
        self.v.append(r)

        self.r = r

        self.theta.append(theta)

        # add recent velocity to delta_v

    def predict_next_box(self, posn=None):
        """
        Predict next bounding box center
        """
        lx, ly = self.path[-1].get_center_coord()
        if len(self.path) == 1:
            return (lx, ly)

        if posn:
            lx, ly = pos
        r = self.v[-1]
        
        # consider acceleration in estimate
        
        if len(self.delta_v) > 1:
          r = r * abs(self.delta_v[-1])

        new_posn = mfn.pol2car((lx, ly), r, self.theta[-1])

        return new_posn

    def get_track_heading(self):
        """
        Accessor for track trajectory information
        Returns the track heading
        """
        return (self.get_last_detection(),self.r, self.delta_v[-1], self.theta[-1])

    def is_alive(self, fc, expiration):
        """
        Check whether a track is expired
        """
        return bool(fc - self.last_frame < expiration)

    def reflect_track(self, reflect_axis=None):
        """
        reflect across an axis
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
        Postprocessing step to construct a linked list
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

    def get_loco_track(self, fdict=None, steps=[]):
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
        for yb in self.path:
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
                    "state_id": fid,
                }
            )
