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


ACCELERATION_THRESHOLD = 1.5
MAX_SCALE_FACTOR = 1.4
MAX_RANGE = 500
MAX_ANGLE = np.pi


class ObjectTrack:
    """
    An abstraction for an object track.
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
        detection_time=0,
        avg_detection_time=1,
        # metrics
        error_over_time=None,
        parent_agent=None,
        path=None,
        predictions=None,
        color=None,
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

        self.color = (color if color != None else rand_color(),)
        self.last_frame = last_frame

    def heartbeat(self):
        """
        Heartbeat function
        """
        self.clock += 1

    def add_new_prediction(self, pred):
        """
        Wrapper for object track adding its own prediction
        """
        pass

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
        avg_window_len = self.parent_agent.obj_tracker.avg_window_len
        return self.estimate_next_position(scale_factor, avg_window_len)

        pass

    def get_state_estimation(self):
        """
        Using filters, estimate the future kinematic state of the target
        using a combination of measurements and predictions
        """
        if len(self.path) > 1:
            return self.predict_next_state()
        return None

        # pass

    def get_track_heading(self):
        """
        Accessor for track trajectory information
        Returns the track heading
        """
        return (
            self.get_last_detection(),
            self.r,
            self.acceleration[-1],
            self.theta[-1],
        )

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
            self.path[i].attributes.next = self.path[i + 1].attributes
            self.path[i + 1].attributes.prev = self.path[i].attributes

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
            return self.path[-1].get_cartesian_coord()
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
            if fdict != None:
                fid = fdict[yb.img_filename]
            # else:
            #     fid = -1
            yb_json = yb.to_json(fid, -1)
            yb_json["track_color"] = self.color[0]
            yb_json["track_id"] = self.track_id
            yb_json["position"] = det.position.to_json()
            steps.append(yb_json)
        return steps

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

        self.error_over_time.append(error)
        self.path.append(detection)

    def update_track_trajectory(self, det, displacement=None):
        """
        Updates the track trajectory components using cartesian coordinates
            Velocity
            Acceleration
            Angular velocity
            Angular Acceleration
        """
        last_det = self.get_last_detection()
        last_pt = last_det.get_cartesian_coord()

        curr_pt = det.get_cartesian_coord()

        theta, distance = mfn.car2pol(last_pt, curr_pt)

        # TODO: Figure out which time is optimal
        # delta_t = max(1,(self.detection_idx[-1] - self.detection_idx[-2]))
        delta_t = self.avg_detection_time

        # self.velocity
        velocity = (distance - self.r_naught[-1]) / delta_t

        # self.acceleration
        acceleration = 0
        jolt = 0
        if len(self.path) > 2:
            acceleration = (velocity - self.delta_r[-1]) / delta_t
            acceleration_sign = 1 if acceleration >= 0 else -1
            acceleration = acceleration_sign * min(
                ACCELERATION_THRESHOLD, abs(acceleration)
            )

        if len(self.path) > 3:
            jolt = (acceleration - self.accel_r[-1]) / delta_t

        self.r_naught.append(distance)
        self.delta_r.append(abs(velocity))

        self.accel_r.append(acceleration)

        self.jolt_r.append(jolt)

        int_theta = theta + 2 * np.pi if theta < 0 else theta
        last_theta = (
            self.theta_naught[-1] + 2 * np.pi
            if self.theta_naught[-1] < 0
            else self.theta_naught[-1]
        )

        # normalize angle to some ratio of pi
        angle_diff = adjust_angle(int_theta - last_theta) / (np.pi)

        # velocity is pi/time
        angular_velocity = angle_diff / delta_t
        angular_acceleration = 0
        angular_jolt = 0
        if len(self.path) > 2:
            angular_acceleration = (angular_velocity - self.delta_theta[-1]) / delta_t
            angular_acceleration_sign = 1 if angular_acceleration >= 0 else -1

            angular_acceleration = angular_acceleration_sign * min(
                ACCELERATION_THRESHOLD, abs(angular_acceleration)
            )

        if len(self.path) > 3:
            angular_jolt = (angular_acceleration - self.accel_theta[-1]) / delta_t

        self.theta_naught.append(theta)
        self.delta_theta.append(angular_velocity)
        self.accel_theta.append(angular_acceleration)
        self.jolt_theta.append(angular_jolt)

        pass

    def estimate_next_position(self, scale_factor=1, avg_window_len=1):
        """
        Estimate position of next detection using trajectory components
        Returns a Detection
        """
        last_pos = self.get_last_detection()

        angle = self.predict_theta(scale_factor, avg_window_len)
        # velocity
        velocity = self.predict_range(scale_factor, avg_window_len)

        pt = mfn.pol2car(
            last_pos.get_cartesian_coord(),
            velocity,
            adjust_angle(adjust_angle(angle)),
        )

        # map cartesian coordinates to sensor coordinates
        pt2 = self.parent_agent.transform_to_local_sensor_coord((0, 0), pt)

        # update sensor yolobox coordinates
        yb = None
        yb = last_pos.get_attributes()
        bbox = [pt2[0], pt2[1], yb.bbox[2], yb.bbox[3]]
        yb.bbox = bbox

        det = Detection(Position(pt[0], pt[1]), yb)
        return det

        pass

    def predict_range(self, scale_factor=1, avg_window_len=1):
        """
        Wrapper for range query
        """
        avg_r = 0 if avg_window_len > 0 else self.r_naught[-1]
        avg_delta_r = 0 if avg_window_len > 0 else self.delta_r[-1]
        avg_accel_r = 0 if avg_window_len > 0 else self.accel_r[-1]
        avg_jolt_r = 0 if avg_window_len > 0 else self.jolt_r[-1]
        add_to_list = lambda input_list, c: input_list[c] if c < len(input_list) else 0

        # smoothing by averaging over the last avg_window_len detections
        for c in range(max(len(self.r_naught) - avg_window_len, 0), len(self.r_naught)):
            age_scale_factor = c / len(self.r_naught)
            avg_r += self.r_naught[c] / (avg_window_len) * age_scale_factor
            avg_delta_r += (
                add_to_list(self.delta_r, c) * 1 / (avg_window_len) * age_scale_factor
            )
            avg_accel_r += (
                add_to_list(self.accel_r, c) * 1 / (avg_window_len) * age_scale_factor
            )
            avg_jolt_r += (
                add_to_list(self.jolt_r, c) * 1 / (avg_window_len) * age_scale_factor
            )

        # r_0 = self.r_naught[-1] # base_len
        r_0 = avg_r
        # delta_r = self.delta_r[-1] # base_len - 1
        delta_r = avg_delta_r
        # accel_r = self.accel_r[-1] # base_len - 2
        accel_r = avg_accel_r
        # jolt_r = self.jolt_r[-1] # base_len - 3
        jolt_r = avg_jolt_r

        # some scale factors to throttle behaviors
        t = scale_factor
        jolt_scale = -0.1
        r = (
            r_0
            + delta_r * self.avg_detection_time
            + (1 / 2) * accel_r * np.square(self.avg_detection_time * t)
            + (1 / 6) * jolt_scale * jolt_r * np.power(self.avg_detection_time * t, 3)
        ) * t
        return min(r, MAX_RANGE)

    def predict_theta(self, scale_factor=1, avg_window_len=1):
        """
        Wrapper for theta query
        """

        avg_theta = 0 if avg_window_len > 0 else self.theta_naught[-1]
        avg_delta_theta = 0 if avg_window_len > 0 else self.delta_theta[-1]
        avg_accel_theta = 0 if avg_window_len > 0 else self.accel_theta[-1]
        avg_jolt_theta = 0 if avg_window_len > 0 else self.jolt_theta[-1]
        add_to_list = lambda input_list, c: input_list[c] if c < len(input_list) else 0

        # smoothing by averaging over the last avg_window_len detections
        for c in range(
            max(len(self.theta_naught) - avg_window_len, 0), len(self.theta_naught)
        ):
            age_scale_factor = c / len(self.theta_naught)
            avg_theta += self.theta_naught[c] / (avg_window_len) * age_scale_factor
            avg_delta_theta += (
                add_to_list(self.delta_theta, c) / (avg_window_len) * age_scale_factor
            )
            avg_accel_theta += (
                add_to_list(self.accel_theta, c) / (avg_window_len) * age_scale_factor
            )
            avg_jolt_theta += (
                add_to_list(self.jolt_theta, c) / (avg_window_len) * age_scale_factor
            )

        # theta_0 = self.theta_naught[-1]
        theta_0 = avg_theta
        # delta_theta = self.delta_theta[-1]
        delta_theta = avg_delta_theta
        # accel_theta = self.accel_theta[-1]
        accel_theta = avg_accel_theta
        # jolt_theta = self.jolt_theta[-1]
        jolt_theta = avg_jolt_theta

        # some scale factors to throttle behaviors
        t = 1
        overall_t = scale_factor
        jolt_scale = -1

        if len(self.path) < 3:
            return self.theta_naught[-1]

        theta = (
            theta_0
            + (
                delta_theta * self.avg_detection_time
                + (1 / 2) * accel_theta * np.square(self.avg_detection_time * t)
                + (1 / 6)
                * jolt_scale
                * jolt_theta
                * np.power(self.avg_detection_time * t, 3)
            )
            * overall_t
            * np.pi
        )

        return theta
