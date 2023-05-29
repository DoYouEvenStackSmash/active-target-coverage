class Target:
    """A class which models a Target which may or may not be covered

    Attributes:
        origin (List[float]): global origin of the target with respect to world frame
        color: color of the target for rendering
        _id (Any): unique identifier for the target

    """

    def __init__(
        self,
        origin,
        color=None,
        _id=0,
    ):
        self.origin = origin
        self.color = color
        self.attributes = origin
        self._id = _id
        # self.path = []

    def reposition(self, destination):
        """
        Repositions the origin of the target to some destination
        """
        self.origin = destination

    def get_origin(self):
        """
        Accessor for the origin of the target
        Returns an (x,y) point
        """
        return self.origin

    def get_attributes(self):
        """
        Accessor for target attributes
        Generally for use in simulation only
        """
        return self.attributes

    def get_id(self):
        """
        Accessor for the unique identifier of the object
        Returns an id
        """
        return self._id
