import collections
import sys
sys.path.append("..")

from aux_functions import *
from YoloBox import YoloBox
from ObjectTrack import ObjectTrack
from categories import CATEGORIES
'''
  Global scope data structure for processing a set of images
  
  global_track_store: {track_id : ObjectTrack}
      lookup dictionary for directly accessing track objects by ID
'''
LABELS = True
IDENTIFIERS = not LABELS
BOXES = IDENTIFIERS
class ObjectTrackManager:
  constants = { "avg_tolerance"   : 10, 
              "track_lifespan"  : 2,
              "default_avg_dist": 10,
              "radial_exclusion": 100,
            }
  display_constants = {"trail_len" : 0}
  def __init__(self,
                global_track_store = {},
                inactive_tracks = [],
                active_tracks = None,
                img_filenames = [],
                annotation_list_fname = "",
                filenames = [],
                sys_paths = [],
                frame_counter = 0,
                layers = [],
                linked_tracks = [],
                trackmap = [],
                fdict = {},
                categories = CATEGORIES,
                img_centers = [],
                imported = False
              ):
    self.global_track_store = global_track_store
    self.inactive_tracks = inactive_tracks
    self.active_tracks = active_tracks
    self.img_filenames = img_filenames
    self.annotation_list_fname = annotation_list_fname
    self.filenames = filenames
    self.sys_paths = sys_paths
    self.frame_counter = frame_counter
    self.layers = layers
    self.linked_tracks = linked_tracks
    self.fdict = fdict
    self.categories = categories
    self.img_centers = img_centers
    self.imported = imported


  def init_new_layer(self):
    '''
    Initialize a new empty layer
    '''
    self.layers.append([])
  

  def add_new_layer(self, yolobox_arr):
    '''
    Add a yolobox array of registered annotations to object track manager as a new layer
    '''
    self.layers.append(yolobox_arr)
  
  def get_layer(self, layer_idx = 0):
    '''
    Accessor for a single layer by index
    returns a layer of yoloboxes
    '''
    if len(self.layers) == 0:
      print()
      return []
    return self.layers[layer_idx]
  

  def add_new_element_to_layer(self, yolobox):
    '''
    Adds a new registered annotation to the latest layer
    '''
    self.layers[-1].append(yolobox)
  
  def add_new_LOCO_annotations_to_layer(self, LOCO_annos):
    '''
    Wrapper calling out to OTFAnnotations ingest
    '''
    self.add_new_layer(OTFAnno.register_new_LOCO_annotations(LOCO_annos))
  

  def get_track(self, track_id):
    ''' 
    Accessor for ObjectTrack entities by track_id 
    '''
    return self.global_track_store[track_id]
  

  def get_category_string(self, class_id):
    '''
    Helper function for accessing category via an integer class_id
    '''
    if class_id >= 0 and class_id < len(self.categories):
      return self.categories[class_id]["name"]
    return "category\nundefined"


  def create_new_track(self, entity, fc):
    '''
    Helper function for creating object tracks
    '''
    track_id = len(self.global_track_store)
    T = ObjectTrack(track_id, entity.class_id)
    T.add_new_step(entity, fc)
    self.global_track_store[track_id] = T
    self.active_tracks.append(T)
  
  
  def initialize_tracks(self, idx = 0):
    '''
    Address special case of initializing object tracks
    '''
    self.active_tracks = collections.deque()
    curr_layer = self.layers[idx]
    for elem in curr_layer:
      self.create_new_track(elem,elem.class_id)
  

  def close_all_tracks(self):
    '''
    Clear the active track queue in preparation for postprocessing
    '''
    if self.active_tracks == None:
      return
    while len(self.active_tracks) > 0:
      self.inactive_tracks.append(self.active_tracks.pop())
  
  
  def link_all_tracks(self, min_len = 0):
    '''
    Resolve linked lists to make tracks externally traversible
    '''
    link_counter = 0
    for k,v in self.global_track_store.items():
      if v.get_step_count() < min_len:
        continue
      link_counter += 1
      self.linked_tracks.append(k)
      self.link_single_track(k)
      # v.path[-1].parent_track = link_counter
    print(f"{link_counter} tracks linked")

  
  def link_single_track(self, track_id):
    '''
    Link a single track by track_id
    '''
    self.global_track_store[track_id].link_path()
  

  def process_all_layers(self):
    '''
    Construct paths through all images
    '''
    for i in range(1,len(self.layers)):
      self.process_layer(i)
  

  def process_layer(self,layer_idx):
    '''
    Update preexisting tracks with a single layer of entities
    '''
    curr_layer = self.layers[layer_idx]
    # if len(curr_layer) == 0:
    #   return
    fc = layer_idx
    pred,pairs = [],[]

    # reinitialize active tracks if there are none currently active
    if (self.active_tracks == None or len(self.active_tracks) == 0):
      if len(curr_layer):
        self.initialize_tracks(layer_idx)
      return
    
    # gather predictions from track heads
    for t in self.active_tracks:
      # if t.is_alive(fc, ObjectTrackManager.constants["track_lifespan"])
      pred.append((t.track_id, t.predict_next_box()))
      # else:

    # create list of all pairs between track heads and curr layer
    for c in range(len(curr_layer)):
      for p in pred:
        d = MathFxns.euclidean_dist(p[1],curr_layer[c].get_center_coord())
        # print("fire")
        pairs.append((p[0], c, d))
    
    sortkey = lambda s: s[2]
    pairs = sorted(pairs,key=sortkey)
    pc,tc,lc = 0,len(self.active_tracks),len(curr_layer)
    # print(lc)
    # print(tc)
    # update existing tracks with new entities
    while tc > 0 and lc > 0 and pc < len(pairs):
      elem = pairs[pc]
      if curr_layer[elem[1]].parent_track != None:
        pc += 1
        continue
      
      '''
        We add a simple check 
      '''
      if elem[2] > ObjectTrackManager.constants["radial_exclusion"]:
        tc-=1
        pc+=1
        continue
      # add entity to closest track
      T = self.global_track_store[elem[0]]
      T.add_new_step(curr_layer[elem[1]], fc)
      # update counters
      tc -= 1
      lc -= 1
      pc += 1
    
    # create new tracks from unused entities
    if lc > 0:
      while lc > 0 and pc < len(pairs):
        elem = pairs[pc]
        if curr_layer[elem[1]].parent_track != None:
          pc += 1
          continue
        # create new ObjectTrack
        self.create_new_track(curr_layer[elem[1]],fc)
        # update counters
        lc -= 1
        pc += 1
    
    if tc > 0:
      # reap tracks which are no longer active
      fc += 1
      max_rot = len(self.active_tracks)
      for i in range(max_rot):
        if self.active_tracks[-1].is_alive(fc, ObjectTrackManager.constants["track_lifespan"]):
          self.active_tracks.rotate()
        else:
          self.inactive_tracks.append(self.active_tracks.pop())
