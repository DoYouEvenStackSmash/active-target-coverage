class State:
  def __init__(self, 
                position = None, # some vector with significance to a reference frame
                orientation = None, # some vector with significance to a reference frame
                time_stamp = 0 # some float with significance to a reference frame
                ):
    self.position = position
    self.orientation = orientation
    self.time_stamp = time_stamp

  def get_position(self):
    '''
    Generic accessor for position vector
    '''
    return self.position
  
  def get_orientation(self):
    '''
    Generic accessor for orientation vector
    '''
    return self.orientation

  def get_time_stamp(self):
    '''
    Generic accessor for time stamp
    '''
    return self.time_stamp

  def to_json(self):
    return {"position": self.position,
            "orientation": self.orientation,
            "time_stamp": self.time_stamp}