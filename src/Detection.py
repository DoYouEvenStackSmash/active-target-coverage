#!/usr/bin/python3
class Detection:
    """An abstraction for describing a detection
    Position(Position): The location of the detection in agent reference frame
    attributes (Yolobox): the attributes of the detection in agent fov reference frame
    """

    def __init__(self, position=None, attributes=None):
        self.position = position if position != None else Position()
        self.attributes = attributes
        self.parent_track = None
        self._id = None

    def get_attributes(self):
        """
        Accessor for attributes
        """
        return self.attributes

    def get_position(self):
        """
        Accessor for position
        """
        return self.position

    def get_cartesian_coord(self):
        """
        Accessor for position's cartesian coordinates
        """
        return self.position.get_center_coord()

    def get_attr_coord(self):
        """
        Accessor for the center coordinate of attributes
        """
        return self.attributes.get_center_coord()

    def set_attributes(self, attr):
        """
        Exposes attributes for modification
        """
        self.attributes = attr


class Position:
    """
    An abstraction for describing a position
    Attributes:
      x,y,z(float,float,float): cartesian coordinates
      theta(float): expansion for polar coordinates
      phi(float): expansion for spherical coordinates
    """

    def __init__(self, x=0, y=0, z=0, theta=0, phi=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi

    def get_center_coord(self):
        return (self.x, self.y)

    def to_json(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "theta": self.theta,
            "phi": self.phi,
        }
