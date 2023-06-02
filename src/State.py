class State:
    """A class representing state of the agent

    Attributes:
        position (List[float]): some vector with significance to a reference frame
        orientation (List[float]): some vector with significance to a reference frame
        time_stamp (float): some float with significance to a reference frame
    """

    def __init__(
        self,
        position=None,  #
        orientation=None,  #
        time_stamp=0,  #
    ):
        self.position = position
        self.orientation = orientation
        self.time_stamp = time_stamp

    def get_position(self):
        """
        Generic accessor for position vector
        """
        return self.position

    def get_orientation(self):
        """
        Generic accessor for orientation vector
        """
        return self.orientation

    def get_time_stamp(self):
        """
        Generic accessor for time stamp
        """
        return self.time_stamp

    def to_json(self):
        return {
            "position": self.position.to_json(),
            "orientation": self.orientation,
            "time_stamp": self.time_stamp,
        }
