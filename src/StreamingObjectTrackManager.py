import collections
import sys

sys.path.append("..")
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from render_support import TransformFxns as tfn

from YoloBox import YoloBox
from ObjectTrack import ObjectTrack
from categories import CATEGORIES
from StreamingAnnotations import StreamingAnnotations as sann
import numpy as np
import cv2
from aux_functions import *

"""
  Global scope data structure for processing a set of images
  
  global_track_store: {track_id : ObjectTrack}
      lookup dictionary for directly accessing track objects by ID
"""


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


LABELS = False
IDENTIFIERS = not LABELS
BOXES = IDENTIFIERS


class ObjectTrackManager:
    constants = {
        "avg_tolerance": 10,
        "track_lifespan": 20,
        "default_avg_dist": 10,
        "radial_exclusion": 100,
    }
    display_constants = {"trail_len": 6}

    def __init__(
        self,
        global_track_store=None,  # Centralized storage for all tracks
        inactive_tracks=None,  # List of inactive tracks
        active_tracks=None,  # Placeholder for deque of active tracks
        img_filenames=None,  # DATA AUGMENTATION: Image filenames for associating detections with images
        annotation_list_fname="",  # Input list of annotations
        filenames=None,  # DATA AUGMENTATION: Filenames for loaded detections
        sys_paths=None,  # DATA AUGMENTATION: Paths to filenames for loaded detections
        frame_counter=0,  # PROCESSING: Clock counter for tracking track lifespan
        layers=None,  # PROCESSING: Accumulator for detections by frame
        linked_tracks=None,  # EXPORT: List of linked tracks for export
        trackmap=None,  # EXPORT: List of identifiers for linked tracks
        fdict=None,  # EXPORT: List of filenames for associating with tracks and their detections
        categories=None,  # Class list for tracks and their identifiers
        img_centers=None,  #
        imported=False,  # Flag denoting whether this is a live tracker or loading track history
        parent_agent=None,  # Placeholder for parent agent
    ):
        self.global_track_store = (
            global_track_store if global_track_store is not None else {}
        )
        self.inactive_tracks = inactive_tracks if inactive_tracks is not None else []
        self.active_tracks = active_tracks
        self.img_filenames = img_filenames if img_filenames is not None else []
        self.annotation_list_fname = annotation_list_fname
        self.filenames = filenames if filenames is not None else []
        self.sys_paths = sys_paths if sys_paths is not None else []
        self.frame_counter = frame_counter
        self.layers = layers if layers is not None else []
        self.linked_tracks = linked_tracks if linked_tracks is not None else []
        self.fdict = fdict if fdict is not None else {}
        self.categories = categories if categories is not None else CATEGORIES
        self.img_centers = img_centers if img_centers is not None else []
        self.imported = imported
        self.parent_agent = parent_agent

    def get_predictions(self, pred_arr):
        """
        Returns the predictions from all active tracks, or a specified index
        """
        if not self.has_active_tracks():
            return []

        for i, trk in enumerate(self.active_tracks):
            if trk.get_state_estimation() == None:
                continue
            pred_arr.append((trk.get_last_detection(), trk.get_state_estimation()))

        return pred_arr

    def add_angular_displacement(self, distance, angle, direction=1):
        """
        Apply an angular displacement to offset a rotation by a parent agent
        """
        pass

    def add_linear_displacement(self, distance, angle, direction=1):
        """
        Apply a linear displacement to offset a translation by a parent agent
        """
        pass

    def init_new_layer(self):
        """
        Initialize a new empty layer
        """
        self.layers.append([])

    def has_active_tracks(self):
        """
        Indicator function for track activity
        returns true if there are active tracks
        """
        return self.active_tracks != None and len(self.active_tracks) > 0

    def add_new_layer(self, yolobox_arr):
        """
        Add a yolobox array of registered annotations to object track manager as a new layer
        """
        self.layers.append(yolobox_arr)

    def get_layer(self, layer_idx=0):
        """
        Accessor for a single layer by index
        returns a layer of yoloboxes
        """
        if len(self.layers) == 0:
            return []
        return self.layers[layer_idx]

    def add_new_element_to_layer(self, yolobox):
        """
        Adds a new registered annotation to the latest layer
        """
        if len(self.layers) == 0:
            self.init_new_layer()
        self.layers[-1].append(yolobox)

    def add_new_LOCO_annotations_to_layer(self, LOCO_annos):
        """
        Wrapper calling out to OTFAnnotations ingest
        """
        self.add_new_layer(OTFAnno.register_new_LOCO_annotations(LOCO_annos))

    def get_track(self, track_id):
        """
        Accessor for ObjectTrack entities by track_id
        """
        return self.global_track_store[track_id]

    def get_category_string(self, class_id):
        """
        Helper function for accessing category via an integer class_id
        """
        if class_id >= 0 and class_id < len(self.categories):
            return self.categories[class_id]["name"]
        return "category\nundefined"

    def create_new_track(self, entity, fc):
        """
        Helper function for creating object tracks
        """
        track_id = len(self.global_track_store)
        T = ObjectTrack(track_id, entity.attributes.class_id)
        T.parent_agent = self.parent_agent
        T.add_new_detection(entity, fc)
        self.global_track_store[track_id] = T
        self.active_tracks.append(T)

    def initialize_tracks(self, idx=0):
        """
        Address special case of initializing object tracks
        """
        self.active_tracks = collections.deque()
        curr_layer = self.layers[idx]
        for elem in curr_layer:
            self.create_new_track(elem, idx)

    def close_all_tracks(self):
        """
        Clear the active track queue in preparation for postprocessing
        """
        if self.active_tracks == None:
            return
        while len(self.active_tracks) > 0:
            self.inactive_tracks.append(self.active_tracks.pop())

    def link_all_tracks(self, min_len=2):
        """
        Resolve linked lists to make tracks externally traversible
        """
        link_counter = 0
        for k, v in self.global_track_store.items():
            if v.get_step_count() <= min_len:
                continue
            link_counter += 1
            self.linked_tracks.append(k)
            self.link_single_track(k)
            # v.path[-1].parent_track = link_counter
        print(f"{link_counter} tracks linked")

    def link_single_track(self, track_id):
        """
        Link a single track by track_id
        """
        self.global_track_store[track_id].link_path()

    def process_all_layers(self):
        """
        Construct paths through all images
        """
        for i in range(1, len(self.layers)):
            self.process_layer(i)

    def process_layer(self, layer_idx):
        """
        Update preexisting tracks with a single layer of entities
        """
        if len(self.layers) == 0:
            return

        curr_layer = self.layers[layer_idx]

        # reinitialize active tracks if there are none currently active
        if not self.has_active_tracks():
            if len(curr_layer):
                self.initialize_tracks(layer_idx)
            return

        fc = layer_idx
        pred, pairs = [], []

        # gather predictions from track heads
        for t in self.active_tracks:
            pred.append((t.track_id, t.get_state_estimation()))

        # create list of all pairs with distances between track heads and detections in curr layer
        for c in range(len(curr_layer)):
            for p in pred:
                print(curr_layer[c].get_cartesian_coordinates())
                d = mfn.spherical_dist(p[1], curr_layer[c])
                # d = mfn.frobenius_dist(p[1], curr_layer[c].get_cartesian_coordinates())
                print(f"distance:{d}")
                pairs.append((p[0], c, d))

        # sort the list of pairs by euclidean distance
        sortkey = lambda s: s[2]
        pairs = sorted(pairs, key=sortkey)

        # number of pairs visited
        pc = 0
        # number of active tracks
        tc = len(self.active_tracks)
        # number of detections in layer
        lc = len(curr_layer)

        # update existing tracks with new entities
        while tc > 0 and lc > 0 and pc < len(pairs):
            elem = pairs[pc]
            if curr_layer[elem[1]].parent_track != None:
                pc += 1
                continue

            # Gate check for global nearest neighbors
            # do not increment pair count for radial exclusion in case this is a new track
            if elem[2] > ObjectTrackManager.constants["radial_exclusion"]:
                tc -= 1
                # pc += 1
                break
                # continue

            # add entity to nearest track
            T = self.global_track_store[elem[0]]
            T.add_new_detection(curr_layer[elem[1]], fc, elem[2])

            # update counters
            tc -= 1
            lc -= 1
            pc += 1

        # create new tracks from unmatched entities
        if lc > 0:
            while lc > 0 and pc < len(pairs):
                elem = pairs[pc]
                if curr_layer[elem[1]].parent_track != None:
                    pc += 1
                    continue

                # create new ObjectTrack
                self.create_new_track(curr_layer[elem[1]], fc)

                # update counters
                lc -= 1
                pc += 1

        if tc > 0:
            fc += 1
            max_rot = len(self.active_tracks)
            for i in range(max_rot):
                # pass over active tracks
                if self.active_tracks[-1].is_alive(
                    fc, ObjectTrackManager.constants["track_lifespan"]
                ):
                    self.active_tracks.rotate()
                else:
                    # reap tracks which are no longer active
                    self.inactive_tracks.append(self.active_tracks.pop())

    def export_loco_fmt(self):
        """
        Export active tracks and associated metadata to loco format
        """
        # construct filename lookup dictionary
        fdict = {}
        # construct "images" : []
        imgs = []
        for i, f in enumerate(self.filenames):
            fdict[f"{f[:-3]}png"] = i
        # construct "images" : []
        imgs = []
        for k, v in fdict.items():
            half_h = 540
            half_w = 960
            # if imported, adjust angles
            if self.imported:
                # if rotaged about the center, swap height and width
                if angle != 0:
                    half_h, half_w = self.img_centers[v]
                else:
                    half_w, half_h = self.img_centers[v]

            h, w = half_h * 2, half_w * 2
            imgs.append({"id": v, "file_name": k, "height": h, "width": w})

        # construct "annotations" : []
        steps = self.export_linked_loco_tracks(fdict)

        """
        Generate new images with which to populate a LOCO of the REFLECTED images
        """

        # construct "linked_tracks" : []
        linked_tracks = [
            {
                "track_id": i,
                "category_id": self.get_track(i).class_id,
                "track_len": 0,
                "steps": [],
            }
            for i in self.linked_tracks
        ]

        trackmap = {}  # {track_id : posn in linked_tracks}
        for i, lt in enumerate(linked_tracks):
            trackmap[linked_tracks[i]["track_id"]] = i

        # add trackmap_index to all annotations
        for s in steps:
            linked_tracks[trackmap[s["track_id"]]]["steps"].append(s["id"])
            s["trackmap_index"] = trackmap[s["track_id"]]

        # add length to linked tracks for fun
        for l in linked_tracks:
            l["track_len"] = len(l["steps"])

        # assemble final dictionary
        exp = {
            "constants": ObjectTrackManager.constants,
            "categories": self.categories,
            "trackmap": list(trackmap),
            "linked_tracks": linked_tracks,
            "images": imgs,
            "annotations": steps,
        }
        return exp

    def export_linked_loco_tracks(self, fdict=None):
        """
        build "annotations" : [] from linked tracks only
        """

        track_steps = []
        for i in self.linked_tracks:
            self.get_track(i).get_loco_track(fdict=None, steps=track_steps)

        for st in range(len(track_steps)):
            bbox = track_steps[st]["bbox"]
            x, y, w, h = bbox

            bbox[0], bbox[1] = x, y
            track_steps[st]["bbox"] = bbox

        for i in range(len(track_steps)):
            track_steps[i]["id"] = i
        return track_steps

    def draw_ybbox_data_on_images(self):
        """
        Draws YoloBox information on the corresponding images
        """
        for layer_idx in range(len(self.layers)):
            img1 = cv2.imread(f"{self.filenames[layer_idx][:-3]}png")

            self.draw_trail(self.layers, layer_idx, img1)
            self.draw_track(self.layers, layer_idx, img1)
            cv2.imwrite(f"{layer_idx}.png", img1)
        # im.save(f"{layer_idx}.jpg")

    def draw_track(self, layer_list, layer_idx, img1, TEXT=False):
        """
        Draw a disappearing track on an image

        "why is it written so inefficiently to iterate over the layers, if there is
        a linked list under the hood?"
        Because we want to write to each picture, layer by layer.
        """
        start = max(layer_idx - ObjectTrackManager.display_constants["trail_len"], 0)
        stop = max(start + 1, layer_idx)
        for trail_idx in range(start, stop):
            layer = layer_list[trail_idx]
            for ybbox in layer:
                color = (255, 0, 0)
                offt = 4
                if ybbox.parent_track != None:  # ybbox is part of a track
                    color = self.get_track(ybbox.parent_track).color
                elif ybbox.next != None:  # bbox is orphaned
                    print("ERROR, Parent is not linked but track is not over.")
                else:  # ybbox is last in the track
                    offt = 50
                    print("what")
                # color = self.get_track(ybbox.parent_track).color
                ArtFxns.draw_line(img1, ybbox, color)

        # add identifier to the entity
        last_layer = layer_list[max(0, layer_idx - 1)]

        for ybbox in last_layer:
            color = (255, 0, 255)

            if ybbox.parent_track != None:
                color = self.get_track(ybbox.parent_track).color

            # label the entities with their id
            if IDENTIFIERS:
                ArtFxns.draw_text(img1, ybbox, color)

            # label the entities with category
            label = "unlabeled"
            if len(self.categories) > 0:
                label = self.get_category_string(int(ybbox.class_id))
            if LABELS:
                ArtFxns.draw_label(img1, ybbox, label, color)

    def draw_trail(self, layer_list, layer_idx, img1):
        """
        Helper function for drawing an individual trail
        """
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
                color = (255, 0, 255)
                offt = 4
                if ybbox.parent_track != None:  # ybbox is part of a track
                    color = self.get_track(ybbox.parent_track).color
                elif ybbox.next != None:  # bbox is orphaned
                    print("ERROR, Parent is not linked but track is not over.")
                    # else: # ybbox is last in the track
                    offt = 4
                    print("last")

                if BOXES:
                    ArtFxns.draw_rectangle(img1, ybbox, color)

    def import_loco_fmt(self, s, sys_path):
        # set up trackmap for accessing tracks
        self.imported = True
        trackmap = s["trackmap"]
        lt = s["linked_tracks"]
        for i, track_id in enumerate(trackmap):
            if track_id not in self.global_track_store or track_id == -1:
                self.global_track_store[track_id] = ObjectTrack(
                    track_id, lt[i]["category_id"]
                )
                self.global_track_store[track_id].class_id = lt[i]["category_id"]
            else:  # already present
                continue

        # load image filenames
        images = s["images"]
        # construct file dict for accessing file ids
        # construct sys_paths list for convenience
        # initialize layers to populate with YoloBoxes
        for i, imf in enumerate(images):
            self.filenames.append(imf["file_name"])
            self.sys_paths.append(sys_path)
            self.fdict[imf["file_name"]] = i
            self.layers.append([])
            self.img_centers.append(
                tuple((int(imf["width"] / 2), int(imf["height"] / 2)))
            )

        # load annotations
        steps = s["annotations"]
        for st in steps:
            # skip step if track is invalid
            if trackmap[st["trackmap_index"]] == -1:
                continue
            track = self.get_track(trackmap[st["trackmap_index"]])
            print(st["image_id"])
            yb = YoloBox(
                track.class_id, st["bbox"], f'{st["image_id"][:-3]}txt', (1920, 1080)
            )
            # print(self.filenames)
            # add YoloBox to the appropriate layer based on the image filename
            self.layers[self.fdict[f"{st['image_id'][:-3]}png"]].append(yb)
            # add the yolobox to the correct track
            track.add_new_step(yb, 0)
