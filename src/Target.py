class Target:
    def __init__(
        self,
        origin,  # global origin of the target wrt world frame
        color=None,  # color of target for rendering
        _id=0,  # unique identifier for the target
    ):
        self.origin = origin
        self.color = color
        self._id = _id

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

    def get_id(self):
        """
        Accessor for the unique identifier of the object
        Returns an id
        """
        return self._id
