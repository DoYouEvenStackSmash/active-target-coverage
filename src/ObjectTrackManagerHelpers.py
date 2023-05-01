#!/usr/bin/python3
from ObjectTrackManager import *
import cv2

class TrackArtFxns:
  def draw_ybbox_data_on_reflected_images(OTM, reflect_axis = None):
    '''
    API accessible prototype image reflection. Does not serialize LOCO
    '''
    for layer_idx in range(len(OTM.layers)):
      img1 = cv2.imread(f"{OTM.filenames[layer_idx][:-3]}png")
      if reflect_axis != None:
        img1 = ImgFxns.reflect_image(img1, reflect_axis)

      OTM.draw_trail(OTM.layers, layer_idx, img1)
      OTM.draw_track(OTM.layers, layer_idx, img1)
      cv2.imwrite(f"reflected_{layer_idx}.png",img1)

  def draw_ybbox_data_on_images(OTM):
    '''
    Draws YoloBox information on the corresponding images
    '''
    for layer_idx in range(len(OTM.layers)):
      img1 = cv2.imread(f"{OTM.filenames[layer_idx][:-3]}png")
      
      OTM.draw_trail(OTM.layers, layer_idx, img1)
      OTM.draw_track(OTM.layers, layer_idx, img1)
      cv2.imwrite(f"{layer_idx}.png",img1)
      # im.save(f"{layer_idx}.jpg")


  def draw_track(OTM,layer_list, layer_idx, img1,TEXT = False):
    '''
    Draw a disappearing track on an image

    "why is it written so inefficiently to iterate over the layers, if there is
    a linked list under the hood?"
    Because we want to write to each picture, layer by layer.
    '''
    start = max(layer_idx- ObjectTrackManager.display_constants["trail_len"],0)
    stop = max(start + 1,layer_idx)
    for trail_idx in range(start,stop):
      layer = layer_list[trail_idx]
      for ybbox in layer:
        color = (255,0,0)
        offt = 4
        if ybbox.parent_track != None:  # ybbox is part of a track
          color = OTM.get_track(ybbox.parent_track).color
        elif ybbox.next != None: # bbox is orphaned
          print("ERROR, Parent is not linked but track is not over.")
        else: # ybbox is last in the track
          offt = 50
          print("what")
        # color = OTM.get_track(ybbox.parent_track).color
        ArtFxns.draw_line(img1,ybbox,color)
    
    # add identifier to the entity
    last_layer = layer_list[max(0, layer_idx - 1)]
    
    for ybbox in last_layer:
      color = (255,0,255)
      
      if ybbox.parent_track != None:
        color = OTM.get_track(ybbox.parent_track).color
      
      # label the entities with their id
      if IDENTIFIERS:
        ArtFxns.draw_text(img1, ybbox, color)
      
      # label the entities with category
      label = "unlabeled"
      if len(OTM.categories) > 0:
        label = OTM.get_category_string(int(ybbox.class_id))
      if LABELS:
        ArtFxns.draw_label(img1, ybbox, label, color)

  def draw_trail(OTM, layer_list, layer_idx, img1):
    '''
    Helper function for drawing an individual trail
    '''
    # draw tracks from all images before
    if layer_idx == 0:
      print("zero")
    start = max(0, layer_idx - 1)
    stop = max(start + 1, layer_idx)
    for trail_idx in range(start, stop):
      layer = layer_list[trail_idx]
      for ybbox in layer:
        if layer_idx == 0:
          print(ybbox.get_corner_coords())
        color = (255,0,255)
        offt = 4
        if ybbox.parent_track != None:  # ybbox is part of a track
          color = OTM.get_track(ybbox.parent_track).color
        elif ybbox.next != None: # bbox is orphaned
          print("ERROR, Parent is not linked but track is not over.")
        else: # ybbox is last in the track
          offt = 4
          print("last")

        if BOXES:
          ArtFxns.draw_rectangle(img1, ybbox, color)