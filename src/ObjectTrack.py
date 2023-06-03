#!/usr/bin/python3
import numpy as np
from aux_functions import *
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from Detection import *
import sys


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


ACCELERATION_THRESHOLD = 1
MAX_SCALE_FACTOR = 1


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

    def __init__(
        self,
        track_id,
        class_id,
        r=0,
        # initial values
        theta_naught=None,
        phi_naught=None,
        r_naught=None,
        # velocities
        delta_theta=None,
        delta_phi=None,
        delta_r=None,
        # accelerations
        accel_theta=None,
        accel_phi=None,
        accel_r=None,
        # impulse
        jolt_theta=None,
        jolt_phi=None,
        jolt_r=None,
        # timekeeping
        clock=0,
        detection_idx=None,
        detection_time=1,
        avg_detection_time=1,
        # metrics
        error_over_time=None,
        parent_agent=None,
        path=None,
        predictions=None,
        color=rand_color(),
        last_frame=-1,
    ):
        """Initializes an ObjectTrack instance.

        Args:
            track_id (int): Unique identifier for the track.
            class_id (int): Track classification for use with an Object Detector.
        """
        self.track_id = track_id
        self.class_id = class_id

        self.r = r

        # initial values
        self.theta_naught = theta_naught if theta_naught != None else [0]
        self.phi_naught = phi_naught if phi_naught != None else [0]
        self.r_naught = r_naught if r_naught != None else [0]

        # velocities
        self.delta_theta = delta_theta if delta_theta != None else [0]
        self.delta_phi = delta_phi if delta_phi != None else [0]
        self.delta_r = delta_r if delta_r != None else [0]

        # accelerations
        self.accel_theta = accel_theta if accel_theta != None else [0]
        self.accel_phi = accel_phi if accel_phi != None else [0]
        self.accel_r = accel_r if accel_r != None else [0]

        self.jolt_theta = jolt_theta if jolt_theta != None else [0]
        self.jolt_phi = jolt_phi if jolt_phi != None else [0]
        self.jolt_r = jolt_r if jolt_r != None else [0]

        # timekeeping
        self.clock = clock
        self.detection_idx = detection_idx if detection_idx != None else [0]
        self.detection_time = detection_time
        self.avg_detection_time = avg_detection_time

        # metrics
        self.error_over_time = error_over_time if error_over_time != None else []

        self.parent_agent = None

        self.path = path if path != None else []
        self.predictions = predictions if predictions != None else []

        self.color = color
        self.last_frame = last_frame

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

        # set detection time
        self.detection_idx.append(self.parent_agent.get_clock())

        if len(self.path):
            self.detection_time = (
                self.detection_idx[-1] - self.detection_idx[-2] + self.detection_time
            )
            self.avg_detection_time = self.detection_time / len(self.detection_idx)
            self.update_track_trajectory(detection)
        else:
            # handle initial condition
            r, y, z = detection.get_cartesian_coordinates()
            theta, phi = detection.get_angles()
            self.r_naught.append(r)
            self.theta_naught.append(
                self.parent_agent.map_detection_back(
                    theta, self.parent_agent.get_fov_width(), self.parent_agent.get_max_x()
                )
            )
            self.phi_naught.append(
                self.parent_agent.map_detection_back(
                    phi, self.parent_agent.get_fov_height(),self.parent_agent.get_max_y()
                )
            )

        self.error_over_time.append(error)
        self.path.append(detection)

    def update_track_trajectory(self, detection, time_interval=None, displacement=None):
        """
        Updates the track trajectory components using cartesian coordinates
        """
        if time_interval == None:
            time_interval = max(1, self.detection_idx[-1] - self.detection_idx[-2])

        theta, phi = detection.get_angles()
        r, y, z = detection.get_cartesian_coordinates()

        # change in position is velocity
        # change in velocity is acceleration
        # change in acceleration is jolt
        self.update_track_r(r, time_interval)
        self.update_track_theta(theta, time_interval)
        self.update_track_phi(phi, time_interval)

    def update_track_r(self, r, time_interval=1):
        """
        Modifier for updating the range element
        """
        r_0 = self.r_naught[-1]
        delta_r_0 = self.delta_r[-1]
        accel_r_0 = self.accel_r[-1]

        delta_r = (r - r_0) / time_interval
        accel_r = min(ACCELERATION_THRESHOLD, (delta_r - delta_r_0) / time_interval)
        jolt_r = (accel_r - accel_r_0) / time_interval

        self.r_naught.append(r)
        self.delta_r.append(delta_r)
        self.accel_r.append(accel_r)
        self.jolt_r.append(jolt_r)

    def update_track_theta(self, theta, time_interval=1):
        """
        Modifier for updating theta element
        """
        theta_0 = self.theta_naught[-1]
        delta_theta_0 = self.delta_theta[-1]
        accel_theta_0 = self.accel_theta[-1]

        max_theta = self.parent_agent.get_fov_width()

        # theta = (theta + max_theta / 2) / max_theta
        theta = self.parent_agent.map_detection_back(
            theta, self.parent_agent.get_fov_width(), self.parent_agent.get_max_x()
        )
        delta_theta = (theta - theta_0) / time_interval
        accel_theta = min(
            ACCELERATION_THRESHOLD, (delta_theta - delta_theta_0) / time_interval
        )
        jolt_theta = (accel_theta - accel_theta_0) / time_interval

        self.theta_naught.append(theta)
        self.delta_theta.append(delta_theta)
        self.accel_theta.append(accel_theta)
        self.jolt_theta.append(jolt_theta)

    def update_track_phi(self, phi, time_interval=1):
        """
        modifier for updating phi element
        """
        phi_0 = self.phi_naught[-1]
        delta_phi_0 = self.delta_phi[-1]
        accel_phi_0 = self.accel_phi[-1]

        max_phi = self.parent_agent.get_fov_height()
        # phi = (phi + max_phi / 2) / max_phi
        phi = self.parent_agent.map_detection_back(
            phi, self.parent_agent.get_fov_height(), self.parent_agent.get_max_y()
        )
        delta_phi = (phi - phi_0) / time_interval
        accel_phi = min(
            ACCELERATION_THRESHOLD, (delta_phi - delta_phi_0) / time_interval
        )
        jolt_phi = (accel_phi - accel_phi_0) / time_interval

        self.phi_naught.append(phi)
        self.delta_phi.append(delta_phi)
        self.accel_phi.append(accel_phi)
        self.jolt_phi.append(jolt_phi)

    def add_new_prediction(self, pred):
        """
        Wrapper for object track adding its own prediction
        """
        pass

    def predict_range(self, scale_factor=1):
        """
        Wrapper for range query
        """
        r_0 = self.r_naught[-1]
        delta_r = self.delta_r[-1]
        accel_r = self.accel_r[-1]
        jolt_r = self.jolt_r[-1]

        r = (
            r_0
            + delta_r * self.avg_detection_time * scale_factor
            + (1 / 2) * accel_r * np.square(self.avg_detection_time * scale_factor)
            + (1 / 6)
            * jolt_r
            * np.power(self.avg_detection_time * scale_factor, 3)
            * scale_factor
            * scale_factor
        )
        return r

    def predict_theta(self, scale_factor=1):
        """
        Wrapper for theta query
        """
        theta_0 = self.theta_naught[-1]
        delta_theta = self.delta_theta[-1]
        accel_theta = self.accel_theta[-1]
        jolt_theta = self.jolt_theta[-1]

        theta = (
            theta_0
            + delta_theta * self.avg_detection_time * scale_factor
            + (1 / 2)
            * accel_theta
            * np.square(self.avg_detection_time * scale_factor)
            * scale_factor
            + (1 / 6)
            * jolt_theta
            * np.power(self.avg_detection_time * scale_factor, 3)
            * scale_factor
            * scale_factor
        )

        return theta

    def predict_phi(self, scale_factor=1):
        """
        Wrapper for phi query
        """
        phi_0 = self.phi_naught[-1]
        delta_phi = self.delta_phi[-1]
        accel_phi = self.accel_phi[-1]
        jolt_phi = self.jolt_phi[-1]

        phi = (
            phi_0
            + delta_phi * self.avg_detection_time * scale_factor
            + (1 / 2)
            * accel_phi
            * np.square(self.avg_detection_time * scale_factor)
            * scale_factor
            + (1 / 6)
            * jolt_phi
            * np.power(self.avg_detection_time * scale_factor, 3)
            * scale_factor
            * scale_factor
        )

        return phi

    def estimate_next_position(self, scale_factor=1):
        """
        Estimate position of next detection using trajectory components
        Returns a Detection
        """
        # predict range
        r = self.predict_range(scale_factor)

        # predict theta
        theta = self.predict_theta(scale_factor)
        theta = theta / self.parent_agent.get_max_x()
        theta = theta * self.parent_agent.get_fov_width() - (
            self.parent_agent.get_fov_width() / 2
        )

        # predict phi
        phi = self.predict_phi(scale_factor)
        phi = phi / self.parent_agent.get_max_y()
        phi = phi * self.parent_agent.get_fov_height() - (
            self.parent_agent.get_fov_height() / 2
        )

        # x = x / 10
        x = self.parent_agent.map_detection_back(
            theta, self.parent_agent.get_fov_width(), 1920
        )
        y = self.parent_agent.map_detection_back(
            phi, self.parent_agent.get_fov_height(), 1080
        )

        last_pos = self.get_last_detection()
        xmax = self.parent_agent.get_max_x()
        ymax = self.parent_agent.get_max_y()
        # some sanity mappping
        x = ((x / xmax) * xmax - xmax / 2) / xmax * 100 + 50
        y = ((y / ymax) * ymax - ymax / 2) / ymax * 100 + 50

        return Detection(Position(r, x, y, theta, phi), last_pos.get_attributes())

    def predict_next_state(self, steps=1):
        """
        Predict an intermediate position of the target using its
        movement characteristics, scaled by the average detection time
        """
        scale_factor = min(
            MAX_SCALE_FACTOR,
            (self.parent_agent.get_clock() - self.detection_idx[-1])
            / self.avg_detection_time,
        )
        return self.estimate_next_position(scale_factor)

        pass

    def get_state_estimation(self):
        """
        Using filters, estimate the future kinematic state of the target
        using a combination of measurements and predictions
        """
        if len(self.path):
            return self.predict_next_state()

        return self.get_last_detection()
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
            
            # fid = None

            fid = yb.img_filename
            yb_json = yb.to_json(fid, self.error_over_time[i], fid, self.color)
            yb_json["track_id"] = self.track_id
            steps.append(yb_json)
        return steps
    
    def add_new_step(self, yb, frame_id):
        ''' 
        Add a new bounding box to the object track
        '''
        # update velocity  
        if len(self.path) > 0:
            pass
            # self.update_track_vector(yb.get_center_coord())
    
        self.last_frame = frame_id
        yb.parent_track = self.track_id
        self.path.append(yb)