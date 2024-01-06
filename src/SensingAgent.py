#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link

import collections

from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from StreamingAnnotations import StreamingAnnotations as sann

import json
from State import State

import sys

from RigidBody import RigidBody
from Sensor import Sensor

import pygame
import time
from typing import Any, List, Dict, Set
from Detection import *


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


class SensingAgent:
    """A class representing a sensing agent.

    Attributes:
        exoskeleton (RigidBody): The rigid body of the agent.
        centered_sensor (Sensor): The default sensor aligned with the axis of rotation for the agent.
        obj_tracker (ObjectTrackManager): The agent's object tracker.
        _id (Any): Unique identifier for the agent.
        sensors (List[Sensor]): List of additional sensors riding on the agent.
        rotation_flag (bool): Flag allowing the agent to rotate.
        translation_flag (bool): Flag allowing the agent to translate.
    """

    def __init__(
        self,
        exoskeleton=None,
        centered_sensor=None,
        obj_tracker=None,
        _id=0,
        sensors=None,
        rotation_flag=True,
        translation_flag=True,
        prediction_flag=True,
        dead_flag=False
    ):
        """Initializes a SensingAgent instance.

        Args:
            exoskeleton (RigidBody): The rigid body of the agent.
            centered_sensor (Sensor): The default sensor aligned with the axis of rotation for the agent.
            obj_tracker (ObjectTrackManager): The agent's object tracker.
            _id (Any): Unique identifier for the agent.
            sensors (List[Sensor]): List of additional sensors riding on the agent.
            rotation_flag (bool): Flag allowing the agent to rotate. Defaults to True.
            translation_flag (bool): Flag allowing the agent to translate. Defaults to True.
        """
        self.exoskeleton = exoskeleton
        self.centered_sensor = centered_sensor
        self.obj_tracker = obj_tracker
        self._id = _id
        self.sensors = sensors if sensors is not None else []
        self.ALLOW_ROTATION = rotation_flag
        self.ALLOW_TRANSLATION = translation_flag
        self.ALLOW_PREDICTION = prediction_flag
        self.IS_DEAD = dead_flag

    def get_clock(self):
        """accessor for clock"""

        return self.exoskeleton.time_stamp

    def set_tolerance(self, tolerance):
        """
        Mutator for centered sensor tolerance
        """
        self.centered_sensor.set_tolerance(tolerance)

    def tracker_query(self):
        """
        Wrapper function for querying the tracker
        Returns a tuple of the estimated rotation and translation to track target
        """
        est_rotation, est_translation = self.estimate_pose_update()
        return (est_rotation, est_translation)

    def reposition(self, est_rotation=None, est_translation=None):
        """
        Wrapper function to trigger a pose update for target coverage
        Returns a tuple of the expected rotation and translation
        """
        rotation, translation = 0, 0

        # perform rotation
        if est_rotation != None:
            rotation = self.apply_rotation_to_agent(est_rotation)

        # perform translation
        if est_translation != None:
            translation = self.apply_translation_to_agent(est_translation)

        # update tracker
        self.tracker_update(rotation, translation)

        return (rotation, translation)

    def tracker_update(self, body_rotation=0, body_translation=0):
        """
        Wrapper function for triggering an update to the active tracks
        Does not return
        """
        if body_rotation != 0:
            direction = 1 if body_rotation > 0 else -1
            self.obj_tracker.add_angular_displacement(0, -body_rotation, direction)

        if body_translation != 0:
            self.obj_tracker.add_linear_displacement(-body_translation, -body_rotation)

    def get_last_detections(self):
        """
        Wrapper function for getting recent detections
        """
        det_arr = self.obj_tracker.get_last_detections()
        pt_arr = []

        for det in det_arr:
            pt_arr.append(self.transform_to_global_coord(det))
        return pt_arr

    def heartbeat(self):
        """
        Exoskeleton heartbeat
        Does not return
        """
        self.exoskeleton.heartbeat()

    def get_origin(self):
        """
        Accessor for the origin of the sensing agent in rigid body
        returns an (x,y) point
        """
        origin = self.exoskeleton.get_center()
        return origin

    def get_fov_theta(self):
        """
        Accessor for the orientation of the sensing agent as the rigid body
        Returns an angle theta
        """
        fov_theta = self.exoskeleton.get_rel_theta()
        return fov_theta

    def get_fov_width(self, sensor_idx=-1):
        """
        Accessor for the width of the fov of the sensing agent
        returns a scalar value
        """
        fov_width = 0
        if sensor_idx != -1:
            fov_width = self.sensors[sensor_idx].get_fov_width()
        else:
            fov_width = self.centered_sensor.get_fov_width()
        return fov_width

    def get_fov_radius(self, sensor_idx=-1):
        """
        Accessor for the range of the sensing agent's sensor
        returns a scalar value
        """
        fov_radius = 0
        if sensor_idx != -1:
            fov_radius = self.sensors[sensor_idx].get_fov_radius()
        else:
            fov_radius = self.centered_sensor.get_fov_radius()
        return fov_radius

    def get_components(self):
        """
        Returns the attributes of an agent
        """
        return (self.exoskeleton, self.centered_sensor)

    def get_center(self):
        """
        Accessor for rotation center of the agent
        """
        return self.exoskeleton.get_center()

    def get_sensor(self, sensor_idx=-1):
        """
        Accessor for the agent's sensor
        """
        if sensor_idx != -1:
            return self.sensors[sensor_idx]
        else:
            return self.centered_sensor

    def get_object_tracker(self):
        """
        Accessor for the agent's object tracker
        """
        return self.obj_tracker

    def translate_agent(self, target_pt):
        """
        Wrapper for translating the rigid body of the agent
        returns a displacement vector (theta, r)
        """
        theta, r = self.exoskeleton.translate_body(target_pt)
        return (theta, r)

    def rotate_agent(self, target_pt, center_line=None):
        """
        Wrapper for rotating the rigid body of the agent
        returns an angle theta
        """
        rotation = 0
        rotation = self.exoskeleton.rotate_body(target_pt)
        return rotation

    def apply_rotation_to_agent(self, rotation):
        """
        Applies a rotation in radians to the exoskeleton
        returns the angle in radians
        """
        if not self.ALLOW_ROTATION:
            return 0

        rotation = self.exoskeleton.apply_rotation_to_body(rotation)
        # update rotation of agent
        self.exoskeleton.rel_theta += rotation
        if self.exoskeleton.rel_theta < -np.pi:
            self.exoskeleton.rel_theta = 2 * np.pi + self.exoskeleton.rel_theta
        if self.exoskeleton.rel_theta > np.pi:
            self.exoskeleton.rel_theta = -2 * np.pi + self.exoskeleton.rel_theta
        return rotation

    def apply_translation_to_agent(self, translation_dist):
        """
        Applies a translation as a vector to the exoskeleton
        returns a vector
        """
        if not self.ALLOW_TRANSLATION:
            return 0

        translation_dist = self.exoskeleton.apply_translation_to_body(translation_dist)
        return translation_dist

    def is_visible(self, target_pt):
        """
        Determines whether a target point is in the Sensor's sensor fov
        Returns true/false
        """
        rotation = self.exoskeleton.get_relative_rotation(target_pt)
        theta, r = mfn.car2pol(self.exoskeleton.get_center(), target_pt)
        if abs(rotation) > self.get_fov_width() / 2:
            return False
        if r > self.get_fov_radius():
            return False
        return True

    def is_detectable(self, target_pt, sensor_id=-1):
        """
        Indicates whether a target point is detectable (within tolerance)
        Returns a boolean indicator and a type identifier
        """
        # boundary conditions
        if sensor_id == -1:
            return self.centered_sensor.is_rel_detectable(target_pt)
        else:
            return self.sensors[sensor_id].is_rel_detectable(target_pt)

    def export_tracks(self):
        """
        Exports the recorded tracks from the agent's object tracker
        returns a LOCO formatted json object
        """
        self.obj_tracker.close_all_tracks()
        self.obj_tracker.link_all_tracks()
        e = self.obj_tracker.export_loco_fmt()
        print(f"exporting states of {self}")
        e["states"] = [s.to_json() for s in self.exoskeleton.states]
        e["sensor_params"] = {
            "fov_radius": self.get_fov_radius(),
            "fov_width": self.get_fov_width(),
        }
        return e

    def new_detection_set(self, frame_id, detection_list):
        """
        Ingest for a new layer of detections from the outside world.

        Creates a new list of yoloboxes associated with current state.
        Adds layer of yoloboxes to tracker and processes layer.
        Does not return
        """
        detections = []
        curr_state = self.exoskeleton.get_age()
        for a in detection_list:
            val = self.transform_to_local_detection_coord(a.get_origin())
            dc = self.transform_to_local_sensor_coord((0, 0), (val[0], val[1]))
            bbx = a.attributes
            bbox = [dc[0], dc[1], 1, 1]
            if bbx != None:
                bbox[2], bbox[3] = bbx[2], bbx[3]
            yb = sann.register_annotation(
                class_id=a.get_id(),
                bbox=bbox,
                img_filename=frame_id,
                # state_id=curr_state,
            )
            posn = Position(val[0], val[1])
            detections.append(Detection(posn, yb))
        self.obj_tracker.filenames.append(frame_id)
        self.obj_tracker.add_new_layer(detections)
        self.obj_tracker.process_layer(len(self.obj_tracker.layers) - 1)

    def add_new_detection(self, frame_id, target_origin):
        """
        Adds a single new detection to a layer
        Does not return
        """
        dc = self.transform_to_local_bbox(target_origin)
        yb = sann.register_annotation(0, dc, frame_id)
        self.obj_tracker.add_new_element_to_layer(yb)

    def transform_to_local_bbox(self, target_pt):
        """
        Calculates detection coordinates relative to Sensor
        returns a Yolo Formatted bbox
        """
        target_rotation = self.exoskeleton.get_relative_rotation(target_pt)
        ratio = target_rotation / self.get_fov_width()

        r = mfn.euclidean_dist(self.get_center(), target_pt)

        x = Sensor.WINDOW_WIDTH * ratio + 50
        y = r
        w = 1
        h = 1

        return [x, y, w, h]

    def transform_to_local_sensor_coord(self, origin, target_pt, sensor_idx=-1):
        """
        Transforms a point to local sensor curved coordinate frame
        """

        theta, r = mfn.car2pol(origin, target_pt)

        ratio = theta / self.get_fov_width()

        x = Sensor.WINDOW_WIDTH * ratio + 50
        # print(x)
        y = r
        w = 1
        h = 1

        return (x, y)

    def transform_to_local_detection_coord(self, target_pt):
        """
        transforms a target point to local rectangular coordinates
        """

        target_rotation = self.exoskeleton.get_relative_rotation(target_pt)
        r = mfn.euclidean_dist(self.get_center(), target_pt)

        local_pt = mfn.pol2car((0, 0), r, target_rotation)
        return local_pt

    def transform_from_local_coord(self, x, y, w=1, h=1):
        """
        Transforms a bbox from sensor local coords to world coords
        returns a point
        """
        theta = (x - 50) / Sensor.WINDOW_WIDTH * self.get_fov_width()
        theta = adjust_angle(self.get_fov_theta() + theta)
        r = y
        pt = mfn.pol2car(self.get_center(), r, theta)

        return pt

    def transform_to_global_coord(self, target_pt):
        """
        Transforms a position from agent local coords to world coords
        returns a point
        """
        # pt1 = self.get_relative_angle()
        # pt1 = position.get_attr_coord()

        pt1 = target_pt
        x, y = pt1
        # print(self.get_rel_theta())
        theta2, r = mfn.car2pol(target_pt, (0, 0))
        theta2 = adjust_angle(theta2 + self.get_fov_theta() + np.pi)

        pt = mfn.pol2car(self.get_center(), r, theta2)
        return pt

    def estimate_pose_update(self, priorities=0):
        """
        Uses past information to predict the next rotation if it exists
        Returns () or the tuple containing the partial rotation
        """
        rel_det = []
        if not self.ALLOW_PREDICTION:
            dets = self.obj_tracker.get_last_detections()
            if not len(dets):
                rel_det = []
            else:
                rel_det.append([(), dets[0]])
        else:
            rel_det = self.estimate_rel_next_detection()

        if not len(rel_det):
            # print("empty")
            return (None, None)

        curr_det, pred_det = rel_det[0]
        if pred_det == None:
            return (None, None)
        pred_pt = pred_det.get_attr_coord()
        # curr_pt = curr_det.get_attr_coord()
        # if no estimate available
        if not len(pred_pt):
            return (None, None)

        # if first element in track, therefore duplicate
        # if curr_pt == pred_pt:
        #     return (None, None)

        status, flag = self.centered_sensor.is_rel_detectable(pred_det.get_attr_coord())

        # if predicted point is detectable from pov of SensingAgent
        if status:
            return (None, None)

        # if predicted point is out of coverage by range
        if flag == Sensor.RANGE:
            pred_pt = pred_det.get_attr_coord()
            offset = pred_pt[1] - (
                self.get_fov_radius() * (1 - self.centered_sensor.tolerance)
            )
            if pred_pt[1] < self.get_fov_radius() * self.centered_sensor.tolerance:
                offset = (
                    pred_pt[1] - self.get_fov_radius() * self.centered_sensor.tolerance
                )
            return (None, offset)

        # if predicted point is out of coverage by angle
        if flag == Sensor.ANGULAR:
            pred_pt = pred_det.get_attr_coord()
            partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
            return (partial_rotation, None)

        # if predicted point is out of coverage by both angle and range
        if flag == Sensor.BOTH:
            offset = pred_pt[1] - (
                self.get_fov_radius() * (1 - self.centered_sensor.tolerance)
            )
            if pred_pt[1] < self.get_fov_radius() * self.centered_sensor.tolerance:
                offset = (
                    pred_pt[1] - self.get_fov_radius() * self.centered_sensor.tolerance
                )
            partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
            return (partial_rotation, offset)

    def get_predictions(self, idx=-1):
        pred = []
        if idx != -1:
            pred = self.obj_tracker.get_predictions(pred)
        else:
            pred = self.obj_tracker.get_predictions(pred)
        return pred

    def estimate_rel_next_detection(self, idx=0):
        """
        Estimates next detection in local coordinate system
        returns a pair of Positions
        """
        pred = self.get_predictions()

        return pred

    def estimate_next_detection(self, idx=0):
        """
        Estimates next detection in external coordinate system
        returns a pair of points
        """

        rel_det = self.estimate_rel_next_detection(idx)
        abs_det = []
        for det in rel_det:
            curr = self.transform_to_global_coord(det[0].get_cartesian_coord())
            pred = self.transform_to_global_coord(det[1].get_cartesian_coord())
            abs_det.append((curr, pred))
        return abs_det
