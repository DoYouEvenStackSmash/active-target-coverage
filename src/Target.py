class Target:
    """A class which models a Target which may or may not be covered

    Attributes:
        origin (List[float]): global origin of the target with respect to world frame
        color: color of the target for rendering
        _id (Any): unique identifier for the target

    """

    def __init__(self, origin, color=None, _id=0, path=None, attributes=None):
        self.origin = origin
        self.color = color
        self.attributes = attributes
        self._id = _id
        self.path = path if path != None else []
        self.idx = 0
        self.frequency = 1

    def step(self):
        """
        Repositions the origin of the target to some destination
        """
        if self.idx + 1 < len(self.path):
            self.idx += 1
            self.origin = self.path[self.idx]
            return True
        return False

    def get_origin(self):
        """
        Accessor for the origin of the target
        Returns an (x,y) point
        """
        return self.origin

    def get_position(self):
        """
        Accessor for the position of the target
        Returns an (x,y) point
        """
        if not len(self.path):
            return self.get_origin()
        return self.path[self.idx]

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
