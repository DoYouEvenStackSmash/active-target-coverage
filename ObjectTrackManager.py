import collections

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
              "track_lifespan"  : 3,
              "default_avg_dist": 10,
              "radial_exclusion": 400,
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

  
  # def import_loco_fmt(self, s, sys_path):
  #   # set up trackmap for accessing tracks
  #   self.imported = True
  #   trackmap = s['trackmap']
  #   lt = s['linked_tracks']
  #   for i,track_id in enumerate(trackmap):
  #     if track_id not in self.global_track_store or track_id == -1:
  #       self.global_track_store[track_id] = ObjectTrack(track_id, lt[i]['category_id'])
  #       self.global_track_store[track_id].class_id = lt[i]['category_id']
  #     else: #already present
  #       continue
    
  #   # load image filenames
  #   images = s['images']
  #   # construct file dict for accessing file ids
  #   # construct sys_paths list for convenience
  #   # initialize layers to populate with YoloBoxes
  #   for i,imf in enumerate(images):
  #     self.filenames.append(imf['file_name'])
  #     self.sys_paths.append(sys_path)
  #     self.fdict[imf['file_name']] = i
  #     self.layers.append([])
  #     self.img_centers.append(tuple((int(imf['width']/2), int(imf['height']/2))))
    
  #   # load annotations
  #   steps = s['annotations']
  #   for st in steps:
  #     # skip step if track is invalid
  #     if trackmap[st['trackmap_index']] == -1:
  #       continue
  #     track = self.get_track(trackmap[st['trackmap_index']])
  #     yb = YoloBox( track.class_id, 
  #                   st['bbox'], 
  #                   f'{self.filenames[st["image_id"]][:-3]}txt',
  #                   self.img_centers[st["image_id"]])
      
  #     # add YoloBox to the appropriate layer based on the image filename
  #     self.layers[self.fdict[self.filenames[st['image_id']]]].append(yb)
  #     # add the yolobox to the correct track
  #     track.add_new_step(yb,0)

  # def export_loco_fmt(self, angle = 0, reflect_axis = None):
  #   '''
  #   Export active tracks and associated metadata to loco format
  #   '''
  #   # construct filename lookup dictionary
  #   fdict = {}
  #   for i,f in enumerate(self.filenames):
  #     fdict[f'{f[:-3]}png'] = i
    
  #   # construct "images" : []
  #   imgs = []
  #   for k,v in fdict.items():
  #     half_h = 540
  #     half_w = 960
  #     # if imported, adjust angles
  #     if self.imported:
  #       #if rotaged about the center, swap height and width
  #       if angle != 0:
  #         half_h, half_w = self.img_centers[v]
  #       else:
  #         half_w, half_h = self.img_centers[v]
        
  #     h,w = half_h * 2, half_w * 2
  #     imgs.append({"id":v, "file_name": k, "height": h, "width": w})
    
  #   # construct "annotations" : []
  #   steps = self.export_linked_loco_tracks(fdict)
    
  #   '''
  #   Generate new images with which to populate a LOCO of the ROTATED images
  #   '''
  #   if angle != 0:
  #     imgs = ImgFxns.rotate_images(imgs, angle)
    
  #   '''
  #   Generate new images with which to populate a LOCO of the REFLECTED images
  #   '''
  #   if reflect_axis != None:
  #     imgs = ImgFxns.reflect_images(imgs, reflect_axis)
    
  #   # construct "linked_tracks" : []
  #   linked_tracks = [{"track_id": i, "category_id" : self.get_track(i).class_id, 
  #                     "track_len": 0, "steps":[] } 
  #                     for i in self.linked_tracks]

    
  #   trackmap = {} # {track_id : posn in linked_tracks}
  #   for i,lt in enumerate(linked_tracks):
  #     trackmap[linked_tracks[i]['track_id']] = i
    
  #   # add trackmap_index to all annotations
  #   for s in steps:
  #     linked_tracks[trackmap[s['track_id']]]['steps'].append(s['id'])
  #     s['trackmap_index'] = trackmap[s['track_id']]
    
  #   # add length to linked tracks for fun
  #   for l in linked_tracks:
  #     l["track_len"] = len(l['steps'])
    
  #   # assemble final dictionary
  #   exp = {
  #           "constants": ObjectTrackManager.constants,
  #           "categories":self.categories,
  #           "trackmap":list(trackmap),
  #           "linked_tracks":linked_tracks,
  #           "images":imgs, 
  #           "annotations":steps
  #         }

  #   return exp

  
  # def export_linked_loco_tracks(self,fdict):  
  #   '''
  #     build "annotations" : [] from linked tracks only
  #   '''
  #   steps = []
  #   for i in self.linked_tracks:
  #     self.get_track(i).get_loco_track(fdict,steps)
  #   # print(f'{len(steps)} total steps')
  #   for i in range(len(steps)):
  #     steps[i]["id"] = i
  #   return steps
  
  # def draw_ybbox_data_on_rotated_images(self, rotation_angle = 0):
  #   '''
  #   API accessible prototype image rotation. Does not serialize LOCO
  #   '''
  #   for layer_idx in range(len(self.layers)):
  #     img1 = cv2.imread(f"{self.filenames[layer_idx][:-3]}png")
  #     if rotation_angle != 0:
  #       img_center = self.img_centers[layer_idx]
  #       img1 = ImgFxns.rotate_image(img1, img_center, rotation_angle)


  #     self.draw_trail(self.layers, layer_idx, img1)
  #     self.draw_track(self.layers, layer_idx, img1)
  #     cv2.imwrite(f"rotated_{layer_idx}.png",img1)

  
  # def draw_ybbox_data_on_reflected_images(self, reflect_axis = None):
  #   '''
  #   API accessible prototype image reflection. Does not serialize LOCO
  #   '''
  #   for layer_idx in range(len(self.layers)):
  #     img1 = cv2.imread(f"{self.filenames[layer_idx][:-3]}png")
  #     if reflect_axis != None:
  #       img1 = ImgFxns.reflect_image(img1, reflect_axis)

  #     self.draw_trail(self.layers, layer_idx, img1)
  #     self.draw_track(self.layers, layer_idx, img1)
  #     cv2.imwrite(f"reflected_{layer_idx}.png",img1)

  # def draw_ybbox_data_on_images(self):
  #   '''
  #   Draws YoloBox information on the corresponding images
  #   '''
  #   for layer_idx in range(len(self.layers)):
  #     img1 = cv2.imread(f"{self.filenames[layer_idx][:-3]}png")
      
  #     self.draw_trail(self.layers, layer_idx, img1)
  #     self.draw_track(self.layers, layer_idx, img1)
  #     cv2.imwrite(f"{layer_idx}.png",img1)
  #     # im.save(f"{layer_idx}.jpg")


  # def draw_track(self,layer_list, layer_idx, img1,TEXT = False):
  #   '''
  #   Draw a disappearing track on an image

  #   "why is it written so inefficiently to iterate over the layers, if there is
  #   a linked list under the hood?"
  #   Because we want to write to each picture, layer by layer.
  #   '''
  #   start = max(layer_idx- ObjectTrackManager.display_constants["trail_len"],0)
  #   stop = max(start + 1,layer_idx)
  #   for trail_idx in range(start,stop):
  #     layer = layer_list[trail_idx]
  #     for ybbox in layer:
  #       color = (255,0,0)
  #       offt = 4
  #       if ybbox.parent_track != None:  # ybbox is part of a track
  #         color = self.get_track(ybbox.parent_track).color
  #       elif ybbox.next != None: # bbox is orphaned
  #         print("ERROR, Parent is not linked but track is not over.")
  #       else: # ybbox is last in the track
  #         offt = 50
  #         print("what")
  #       # color = self.get_track(ybbox.parent_track).color
  #       ArtFxns.draw_line(img1,ybbox,color)
    
  #   # add identifier to the entity
  #   last_layer = layer_list[max(0, layer_idx - 1)]
    
  #   for ybbox in last_layer:
  #     color = (255,0,255)
      
  #     if ybbox.parent_track != None:
  #       color = self.get_track(ybbox.parent_track).color
      
  #     # label the entities with their id
  #     if IDENTIFIERS:
  #       ArtFxns.draw_text(img1, ybbox, color)
      
  #     # label the entities with category
  #     label = "unlabeled"
  #     if len(self.categories) > 0:
  #       label = self.get_category_string(int(ybbox.class_id))
  #     if LABELS:
  #       ArtFxns.draw_label(img1, ybbox, label, color)

  # def draw_trail(self, layer_list, layer_idx, img1):
  #   '''
  #   Helper function for drawing an individual trail
  #   '''
  #   # draw tracks from all images before
  #   if layer_idx == 0:
  #     print("zero")
  #   start = max(0, layer_idx - 1)
  #   stop = max(start + 1, layer_idx)
  #   for trail_idx in range(start, stop):
  #     layer = layer_list[trail_idx]
  #     for ybbox in layer:
  #       if layer_idx == 0:
  #         print(ybbox.get_corner_coords())
  #       color = (255,0,255)
  #       offt = 4
  #       if ybbox.parent_track != None:  # ybbox is part of a track
  #         color = self.get_track(ybbox.parent_track).color
  #       elif ybbox.next != None: # bbox is orphaned
  #         print("ERROR, Parent is not linked but track is not over.")
  #       else: # ybbox is last in the track
  #         offt = 4
  #         print("last")

  #       if BOXES:
  #         ArtFxns.draw_rectangle(img1, ybbox, color)

  def rotate_linked_tracks(self, offset_degrees):
    '''
    API accessible rotation of bounding boxes
    '''
    for i in self.linked_tracks:
      self.get_track(i).rotate_track(offset_degrees)
    
  def reflect_linked_tracks(self, reflect_axis):
    for i in self.linked_tracks:
      self.get_track(i).reflect_track(reflect_axis)

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
  
  
  def initialize_tracks(self):
    '''
    Address special case of initializing object tracks
    '''
    self.active_tracks = collections.deque()
    curr_layer = self.layers[0]
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
    fc = layer_idx
    pred,pairs = [],[]
    
    # gather predictions from track heads
    for t in self.active_tracks:
      # if t.is_alive(fc, ObjectTrackManager.constants["track_lifespan"])
      pred.append((t.track_id, t.predict_next_box()))
      # else:

    # create list of all pairs between track heads and curr layer
    for c in range(len(curr_layer)):
      for p in pred:
        d = MathFxns.euclidean_dist(p[1],curr_layer[c].get_center_coord())
        pairs.append((p[0], c, d))
    
    sortkey = lambda s: s[2]
    pairs = sorted(pairs,key=sortkey)
    # print(pairs)
    pc,tc,lc = 0,len(self.active_tracks),len(curr_layer)
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
