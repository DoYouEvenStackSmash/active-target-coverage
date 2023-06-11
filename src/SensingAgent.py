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


DEFAULT_RANGE = 10


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

    def get_clock(self):
        """accessor for clock"""

        return self.exoskeleton.time_stamp

    def heartbeat(self):
        """
        Exoskeleton heartbeat
        Does not return
        """
        self.exoskeleton.heartbeat()

    def get_origin(self):
        """
        Accessor for the origin of the sensing agent in rigid body
        returns a Position()
        """
        origin = self.exoskeleton.get_center()
        return origin

    def get_components(self):
        """
        Returns the attributes of an agent
        """
        return (self.exoskeleton, self.centered_sensor)

    def estimate_pose_update(self, priorities=0):
        """
        Uses past information to predict the next rotation if it exists
        Returns () or the tuple containing the partial rotation
        """

        rel_det = self.estimate_rel_next_detection()

        if not len(rel_det):
            print("empty")
            return (None, None)

        curr_det, pred_det = rel_det[0]
        if pred_det == None:
            return (None, None)
        pred_pt = pred_det.get_attr_coord()
        curr_pt = curr_det.get_attr_coord()
        print(pred_pt)
        # if no estimate available
        if not len(pred_pt):
            return (None, None)

        # if first element in track, therefore duplicate
        # if curr_pt == pred_pt:
        #     return (None, None)

        # status, flag = self.centered_sensor.is_rel_detectable(pred_det.get_attr_coord())
        status, flag = self.centered_sensor.is_rel_detectable_fov(pred_det)
        # if predicted point is detectable from pov of SensingAgent
        if status:
            return (None, None)

        # if predicted point is out of coverage by range
        if flag == Sensor.RANGE:
            pred_pt = pred_det.get_attr_coord()
            print(f"pred_pt {pred_pt}")
            offset = pred_pt[1] - (self.get_fov_radius() * (1 - Sensor.TOLERANCE))
            if pred_pt[1] < self.get_fov_radius() * Sensor.TOLERANCE:
                offset = pred_pt[1] - self.get_fov_radius() * Sensor.TOLERANCE
            return (None, offset)

        # if predicted point is out of coverage by angle
        if flag == Sensor.ANGULAR:
            pred_pt = pred_det.get_attr_coord()
            partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
            return (partial_rotation, None)

        # if predicted point is out of coverage by both angle and range
        if flag == Sensor.BOTH:
            offset = pred_pt[1] - (self.get_fov_radius() * (1 - Sensor.TOLERANCE))
            if pred_pt[1] < self.get_fov_radius() * Sensor.TOLERANCE:
                offset = pred_pt[1] - self.get_fov_radius() * Sensor.TOLERANCE
            partial_rotation = (pred_pt[0] - 50) / 100 * self.get_fov_width()
            return (partial_rotation, offset)

    def tracker_query(self):
        """
        Wrapper function for querying the tracker
        Returns a tuple of the estimated rotation and translation to track target
        """
        est_rotation, est_translation = self.estimate_pose_update()
        return (est_rotation, est_translation)

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

    # sensor functions
    def get_sensor(self, sensor_idx=-1):
        """
        Accessor for the agent's sensor
        """
        if sensor_idx != -1:
            return self.sensors[sensor_idx]
        else:
            return self.centered_sensor

    def is_visible(self, target_posn):
        """
        Determines whether a target point is in the Sensor's sensor fov
        Returns true/false
        """
        rotation = self.exoskeleton.get_relative_rotation(target_posn)
        print(rotation)
        theta, r = mfn.car2pol(
            self.exoskeleton.get_center().get_cartesian_coordinates(),
            target_posn.get_cartesian_coordinates(),
        )
        if abs(rotation) > self.get_fov_width() / 2:
            return False
        if r > self.get_fov_radius():
            return False
        return True

    def is_visible_fov(self, theta, phi, dist):
        """
        Determines whether target angle and range will be in sensor fov
        """
        if abs(theta) > self.get_fov_width() / 2:
            return False
        if abs(phi) > self.get_fov_height() / 2:
            return False
        if dist > self.get_fov_radius():
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

    def is_detectable_fov(self, target_posn, sensor_id=-1):
        """
        Indicates whether a target point is detectable (within tolerance)
        Returns a boolean indicator and a type identifier
        """
        # boundary conditions
        if sensor_id == -1:
            return self.centered_sensor.is_rel_detectable_fov(target_posn)
        else:
            return self.sensors[sensor_id].is_rel_detectable_fov(target_posn)

    def get_max_x(self):
        """
        Accessor for maximum horizontal fov
        """
        return self.centered_sensor.get_max_x()

    def get_max_y(self):
        """
        Accessor for maximum vertical fov
        """
        return self.centered_sensor.get_max_y()

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

    def get_fov_height(self, sensor_idx=-1):
        """
        Accessor for the width of the fov of the sensing agent
        returns a scalar value
        """
        fov_height = 0
        if sensor_idx != -1:
            fov_height = self.sensors[sensor_idx].get_fov_height()
        else:
            fov_height = self.centered_sensor.get_fov_height()
        return fov_height

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

    # exoskeleton functions

    def get_center(self):
        """
        Accessor for rotation center of the agent
        """
        return self.exoskeleton.get_center()

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
            print(f"est_translation:{est_translation}")
            translation = self.apply_translation_to_agent(est_translation)

        # update tracker
        self.tracker_update(rotation, translation)

        return (rotation, translation)

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

    # Tracker functions

    def get_object_tracker(self):
        """
        Accessor for the agent's object tracker
        """
        return self.obj_tracker

    def get_predictions(self, idx=-1):
        """
        Accessor for object tracker predictions
        """
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
            curr = self.transform_from_det_coord(det[0])
            pred = self.transform_from_det_coord(det[1])
            abs_det.append((curr, pred))
        return abs_det

    # legacy ingest for detections
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
            # print(f"val {val}")
            dc = self.transform_to_local_sensor_coord(Position(0, 0, 0), val)
            print(dc)
            x, y, z = dc.get_cartesian_coordinates()
            bbox = [x, y, 1, 1]
            yb = sann.register_annotation(a.get_id(), bbox, curr_state)

            detections.append(Detection(val, yb))
        return detections

    # ingest
    def ingest_new_yolobox_layer(self, yolobox_layer=None, SCALE_FLAG=False):
        """
        wrapper for processing a new layer of detections
        """
        yolobox_layer = yolobox_layer if yolobox_layer != None else []

        detections = self.create_detection_layer_from_yoloboxes(
            yolobox_layer, SCALE_FLAG
        )

        self.heartbeat()

        self.obj_tracker.add_new_layer(detections)
        self.obj_tracker.process_layer(len(self.obj_tracker.layers) - 1)

    def create_detection_layer_from_yoloboxes(
        self, yolobox_layer=None, SCALE_FLAG=False
    ):
        """
        Create a detection layer from yoloboxes
        """
        yolobox_layer = yolobox_layer if yolobox_layer != None else []

        curr_state = self.get_clock()
        detection_layer = []
        for i in range(len(yolobox_layer)):
            yb = yolobox_layer[i]
            detection_layer.append(self.create_detection(yb, SCALE_FLAG))

        return detection_layer

    def create_detection(self, yb, SCALE_FLAG=False):
        """
        Wrapper for create detection
        """
        x, y, w, h = yb.bbox
        distance = yb.distance

        if SCALE_FLAG:
            x, w = x / self.get_max_x(), w / self.get_max_x()
            y, h = y / self.get_max_y(), h / self.get_max_y()

        posn = self.create_position(x, y, w, h, distance)
        det = Detection(posn, yb)
        return det

    def create_position(self, x, y, w, h, distance=0):
        """
        wrapper for Yolo style detections

        # calculate vector 1, agent pov on horizontal plane

                    (0,0)
                +-------+-------+
                |       |       |
                |       |       |
                |       |       |
        (0,0)   |__ B<--A_______| (100,0) * theta
                |   |  /|       |
                |   v / |       |
                |   C   |       |
                +-------+-------+
                    (0,100) * phi
        Agent frame of reference
        image_shape: 1000
        sensor_fov_width: pi / 2
        rel_x = 25
        theTa = -np.pi / 4
        """
        dist = distance
        # normalize x between 0 and 100
        rel_x = (
            x * self.get_max_x() - (self.get_max_x() / 2)
        ) / self.get_max_x() * 100 + 50
        # normalize theta in terms of agent pov
        theta = (rel_x / 100) * self.get_fov_width() - (self.get_fov_width() / 2)

        # vertical component
        rel_y = (
            y * self.get_max_y() - (self.get_max_y() / 2)
        ) / self.get_max_y() * 100 + 50
        phi = (rel_y / 100) * self.get_fov_height() - (self.get_fov_height() / 2)

        posn = Position(dist, rel_x, rel_y, theta, phi)
        return posn

    # legacy coordinate transform
    def transform_to_local_frame_coord(self, target_point, sensor_idx=-1):
        """
        Transforms a point to local sensor curved coordinate frame
        """

        # target_pt = target_pt.ge
        # origin = origin.get_cartesian_coordinates()
        x, y, z = target_point
        rel_x = x / self.get_max_x() * 100
        rel_y = y / self.get_max_y() * 100

        theta = rel_x / 100 * self.get_fov_width() - (self.get_fov_width() / 2)

        phi = rel_y / 100 * self.get_fov_height() - (self.get_fov_height() / 2)

        dist = x

        posn = Position(dist, rel_x, rel_y, theta, phi)
        # print(f"frame_coords {posn.get_cartesian_coordinates()}")
        return posn

    # legacy detection transform
    def transform_to_local_detection_coord(self, target_pt):
        """
        transforms a target point to local rectangular coordinates
        """

        target_rotation = self.exoskeleton.get_relative_rotation(target_pt)
        r = mfn.euclidean_dist(
            self.get_center().get_cartesian_coordinates(),
            target_pt.get_cartesian_coordinates(),
        )

        local_pt = mfn.pol2car((0, 0, 0), r, target_rotation)
        x, y, z = local_pt
        return Position(x, y, z)

    # legacy bbox transform back to global coordinate
    def transform_from_local_coord(self, x, y, w=1, h=1):
        """
        Transforms a bbox from sensor local coords to world coords
        returns a point
        """
        theta = (x - 50) / Sensor.WINDOW_WIDTH * self.get_fov_width()
        theta = adjust_angle(self.get_fov_theta() + theta)
        r = y
        pt = mfn.pol2car(self.get_center().get_cartesian_coordinates(), r, theta)

        return pt

    def map_detection_back(self, angle, max_angle, max_coord=1920):
        """
        maps an angle back to a scalar
        """
        ratio = angle / max_angle
        val = ratio * max_coord + max_coord / 2
        return val

    def transform_from_det_coord(self, target_pt):
        """
        Transforms a position from sensor frame coordinates to frame coordinates
        returns a point
        """
        x, y, z = target_pt.get_cartesian_coordinates()
        theta, phi = target_pt.get_angles()
        y = self.map_detection_back(theta, self.get_fov_width(), self.get_max_x())
        z = self.map_detection_back(phi, self.get_fov_height(), self.get_max_y())

        return Position(x, y, z, theta, phi)

    def transform_from_frame_coord(self, frame_posn):
        """
        Transforms a position from frame coordinates to agent coordinates
        returns a point
        """
        r, y, z = frame_posn.get_cartesian_coordinates()
        theta, phi = frame_posn.get_angles()
        x,y,z = mfn.pol2car((0,0,0), r, theta)
        return Position(x,y,z,theta, phi)
        

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
