#!/usr/bin/python3
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from aux_functions import *
import time
from Detection import *
from State import State


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


class RigidBody:
    """A class representing a rigid body.

    Attributes:
        LINE_LEN (int): Length constant for the rigid body.
        parent_agent: Placeholder for the parent agent of the rigid body.
        rigid_link: Underlying Link object and implementation for transformations.
        ref_origin (Tuple[float, float]): Global origin of the rigid body in the agent's world frame.
        endpoint (Tuple[float, float, float]): Global coordinate of the end point in the agent's world frame.
        ref_center (Tuple[float, float, float]): Global coordinate of the center point (central sensor) in the agent's world frame.
        point_set (List[Any]): Placeholder for the points in the rigid body. Superseded by rigid_link.
        rel_theta (float): Global angle theta describing where the agent is oriented in the agent's world frame.
        color: Color for rendering agent details.
        time_stamp (float): A float with significance to a reference clock.
        states (List[Any]): List of rigid body states.
    """

    LINE_LEN = 30

    def __init__(
        self,
        parent_agent=None,
        rigid_link=None,
        ref_origin=(0, 0),
        endpoint=(0, 0, 0),
        ref_center=(0, 0, 0),
        point_set=None,
        rel_theta=0,
        color=None,
        time_stamp=0,
        states=None,
    ):
        """Initializes a RigidBody instance.

        Args:
            parent_agent: Placeholder for the parent agent of the rigid body.
            rigid_link: Underlying Link object and implementation for transformations.
            ref_origin (Tuple[float, float]): Global origin of the rigid body in the agent's world frame.
            endpoint (Tuple[float, float, float]): Global coordinate of the end point in the agent's world frame.
            ref_center (Tuple[float, float, float]): Global coordinate of the center point (central sensor) in the agent's world frame.
            point_set (List[Any], optional): Placeholder for the points in the rigid body. Superseded by rigid_link.
            rel_theta (float, optional): Global angle theta describing where the agent is oriented in the agent's world frame. Defaults to 0.
            color: Color for rendering agent details.
            time_stamp (float, optional): A float with significance to a reference clock. Defaults to 0.
            states (List[Any], optional): List of rigid body states.
        """
        self.parent_agent = parent_agent
        self.origin = ref_origin
        self.endpoint = endpoint
        self.ref_center = ref_center
        self.point_set = point_set if point_set is not None else []
        self.body = rigid_link
        self.rel_theta = rel_theta
        self.color = color if color != None else rand_color()
        self.time_stamp = time_stamp
        self.states = states if states is not None else []

    def heartbeat(self):
        """
        Internal heartbeat. Records a new state.
        Does not return
        """
        self.increment_clock()

        new_state = State(
            self.get_as_posn(self.get_center()), self.get_rel_theta(), self.get_clock()
        )

        self.states.append(new_state)

    def increment_clock(self):
        """
        Accumulator for internal clock
        Does not return
        """
        self.time_stamp += 1

    def get_clock(self):
        """
        Accessor for internal clock
        returns a float
        """
        return self.time_stamp

    def get_last_state(self):
        """
        Accessor for states buffer
        Returns a State
        """
        return self.states[-1]

    def get_age(self):
        """
        Alternative accessor to state list
        Returns an integer
        """
        return len(self.states)

    def get_rel_theta(self):
        """
        Accessor for relative angle
        returns an angle theta in radians
        """
        return self.rel_theta

    def get_body(self):
        """
        Accessor for internal polygon
        Returns a polygon object or None
        """
        return self.body

    def get_points(self):
        """
        Wrapper for points access
        """
        if self.body == None:
            return self.get_point_set()
        return self.get_body_points()

    def get_point_set(self):
        """
        Accessor for the point set which is constrained to the RigidBody
        Returns a list of points
        """
        return self.point_set

    def get_body_points(self):
        """
        Get internal point set
        returns a list of points
        """
        return self.body.dump_points()

    def get_relative_angle(self):
        """
        Accessor for the RigidBody's angle relative to previous RigidBody
        Returns an angle theta
        """
        return self.rel_theta

    def get_as_posn(self, coords):
        """
        convenience wrapper for getting the coordinates as a Position
        """
        return Position(coords[0], coords[1])

    def get_center(self):
        """
        Accessor for the RigidBody's origin
        returns a point
        """
        return self.ref_center

    def get_endpoint(self):
        """
        Accessor for the RigidBody endpoint
        Returns a point
        """
        return self.endpoint

    def get_origin(self):
        """
        Accessor for an origin point
        returns a point
        """
        return self.origin

    def get_normals(self):
        """
        Calculates coordinate axes x,y in R2
        Returns a pair of points representing axis unit endpoints
        """
        ox, oy = self.get_center()
        theta = self.rel_theta
        xx, xy = RigidBody.LINE_LEN * np.cos(theta), RigidBody.LINE_LEN * np.sin(theta)
        yx, yy = RigidBody.LINE_LEN * np.cos(
            theta + np.pi / 2
        ), RigidBody.LINE_LEN * np.sin(theta + np.pi / 2)
        return ((xx + ox, xy + oy), (yx + ox, yy + oy))

    def get_relative_rotation(self, target_point):
        """
        Given a target point, computes the angle theta between the current
        endpoint and the target point for use during rotation

        USES GLOBAL COORDINATES

        Returns a normalized angle theta
        """

        norm, dist = mfn.car2pol(self.get_center(), self.get_endpoint())
        rad, r = mfn.car2pol(self.get_center(), target_point)

        norm = mfn.correct_angle(norm)
        rad = mfn.correct_angle(rad)

        rotation = np.subtract(rad, norm)

        # correction for arctan identification
        if rotation > np.pi:
            rotation = rotation - 2 * np.pi
        if rotation < -np.pi:
            rotation = rotation + 2 * np.pi

        return rotation

    def rotate_body(self, target_point):
        """
        Rotation for the rigid body
        """
        rotation = self.get_relative_rotation(target_point)
        rotation = self.apply_rotation_to_body(rotation)
        self.rel_theta += rotation
        return rotation

    def translate_body(self, target_point):
        """
        Translates internal polygon
        Does not return
        """
        theta, r = mfn.car2pol(self.get_center(), target_point)
        pt2 = mfn.pol2car(self.get_center(), r, theta)
        cx, cy = self.get_center()

        x_disp, y_disp = pt2[0] - cx, pt2[1] - cy
        self.endpoint = mfn.pol2car(self.endpoint, r, theta)
        self.origin = mfn.pol2car(self.origin, r, theta)
        self.ref_center = mfn.pol2car(self.ref_center, r, theta)

        translate_polygon(self.body, x_disp, y_disp)
        return (theta, r)

    def apply_rotation_to_body(self, rotation):
        """
        Applies a rotation to internal polygon
        Does not return
        """
        rot_mat = tfn.calculate_rotation_matrix(rotation, 1)
        self.endpoint = tfn.rotate_point(
            self.get_center(), self.get_endpoint(), rot_mat
        )
        self.origin = tfn.rotate_point(self.get_center(), self.get_origin(), rot_mat)
        self.ref_center = tfn.rotate_point(
            self.get_center(), self.get_center(), rot_mat
        )

        rotate_polygon(self.body, rot_mat, self.get_center())
        return rotation

    def apply_translation_to_body(self, translation_dist=0):
        """
        Applies a translation to the rigid body
        returns a displacement vector
        """
        self.ref_center = mfn.pol2car(
            self.get_center(), translation_dist, self.get_rel_theta()
        )
        self.endpoint = mfn.pol2car(
            self.get_endpoint(), translation_dist, self.get_rel_theta()
        )
        self.origin = mfn.pol2car(
            self.get_origin(), translation_dist, self.get_rel_theta()
        )
        x_disp, y_disp = mfn.pol2car((0, 0), translation_dist, self.get_rel_theta())
        translate_polygon(self.body, x_disp, y_disp)
        return translation_dist

    def update_orientation(self, point_set, theta):
        """
        TODO rotate by angle
        """
        self.point_set = point_set
        self.rel_theta = theta

    def update_point_set(self, point_set):
        """
        Replace existing point set with a new point set.
        Used during rotations
        Does not return
        """
        self.point_set = point_set

    def get_grid(self, spacing=10, radius=500):
        """
        Drawing function for getting a coordinate grid oriented with the rigid body
        returns a List of (pt1, pt2) points
        """
        r = radius
        tlpt = mfn.pol2car(
            self.get_center(), r, adjust_angle(self.get_rel_theta() - np.pi / 4)
        )
        trpt = mfn.pol2car(
            self.get_center(), r, adjust_angle(self.get_rel_theta() + np.pi / 4)
        )
        blpt = mfn.pol2car(
            self.get_center(), r, adjust_angle(self.get_rel_theta() - 3 * np.pi / 4)
        )
        horiz_theta, horiz_r = mfn.car2pol(tlpt, trpt)
        vert_theta, vert_r = mfn.car2pol(tlpt, blpt)

        axes = []
        step = vert_r / spacing
        for i in range(spacing + 1):
            lpt = mfn.pol2car(tlpt, step * i, vert_theta)
            rpt = mfn.pol2car(trpt, step * i, vert_theta)
            axes.append([lpt, rpt])
        for i in range(spacing + 1):
            lpt = mfn.pol2car(tlpt, step * i, horiz_theta)
            rpt = mfn.pol2car(blpt, step * i, horiz_theta)
            axes.append([lpt, rpt])
        return axes

    def get_horizontal_axis(self, off_t=0):
        """
        Helper function for getting a horizontal line through the center of the agent
        Returns a list of (pt1, pt2) points
        """
        curr_rotation = self.get_rel_theta()
        pos_x = [
            mfn.pol2car(
                self.get_center(), 100, adjust_angle(curr_rotation + np.pi / 2)
            ),
            mfn.pol2car(self.get_center(), 40, adjust_angle(curr_rotation - np.pi / 2)),
        ]
        return pos_x

    def get_vertical_axis(self, off_t=0):
        """
        Helper function for getting a vertical line through the center of the agent
        Returns a list of (pt1, pt2) points
        """
        curr_rotation = self.get_rel_theta()
        pos_x = [
            mfn.pol2car(self.get_center(), 100, curr_rotation),
            mfn.pol2car(self.get_center(), 40, adjust_angle(curr_rotation + np.pi)),
        ]
        return pos_x
