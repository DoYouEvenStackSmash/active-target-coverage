import collections
import sys

sys.path.append("..")
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from render_support import TransformFxns as tfn
from aux_functions import *
from YoloBox import YoloBox
from ObjectTrack import ObjectTrack
from categories import CATEGORIES

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


LABELS = True
IDENTIFIERS = not LABELS
BOXES = IDENTIFIERS


class ObjectTrackManager:
    constants = {
        "avg_tolerance": 10,
        "track_lifespan": 2,
        "default_avg_dist": 10,
        "radial_exclusion": 300,
    }
    display_constants = {"trail_len": 0}

    def __init__(
        self,
        global_track_store={},  # Centralized storage for all tracks
        inactive_tracks=[],  # list of inactive tracks
        active_tracks=None,  # placeholder for deque of active tracks
        img_filenames=[],  # DATA AUGMENTATION: image filenames for associating detections with images
        annotation_list_fname="",  # input list of annotations
        filenames=[],  # DATA AUGMENTATION: filenames for loaded detections
        sys_paths=[],  # DATA AUGMENTATION: paths to filenames for loaded detections
        frame_counter=0,  # PROCESSING: clock counter for tracking track lifespan
        layers=[],  # PROCESSING: accumulator for detections by frame
        linked_tracks=[],  # EXPORT: list of linked tracks for export
        trackmap=[],  # EXPORT: list of identifiers for linked tracks
        fdict={},  # EXPORT: List of filenames for associating with tracks and their detections
        categories=CATEGORIES,  # class list for tracks and their identifiers
        img_centers=[],  #
        imported=False,  # flag denoting whether this is a live tracker or loading track history
        parent_agent=None,  # placeholder for parent agent
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
        self.parent_agent = parent_agent

    def get_predictions(self):
        """
        Get estimates from all active tracks
        """
        if not self.has_active_tracks():
            return []
        estimates = []
        for i in range(len(self.active_tracks)):
            trk = self.active_tracks[i]
            estimates.append(trk.predict_next_box())
        return estimates

    def add_angular_displacement(self, distance, angle, direction=1):
        """
        Apply an angular displacement to offset a rotation by a parent agent
        """
        if not self.has_active_tracks():
            return

        for i in range(len(self.active_tracks)):
            trk = self.active_tracks[i]
            last_d, last_v, delta_v, theta = trk.get_track_heading()

            disp = angle / self.parent_agent.get_fov_width() * 100

            if disp < 0:
                trk.theta[-1] = adjust_angle(trk.theta[-1] + angle + np.pi / 2)
            if disp > 0:
                if trk.theta[-1] < -np.pi / 2:
                    trk.theta[-1] = adjust_angle(trk.theta[-1] - angle + np.pi / 2)
                else:
                    trk.theta[-1] = adjust_angle(trk.theta[-1] + angle - np.pi / 2)

            x, y = last_d
            nx, ny = [last_d[0] + disp, last_d[1]]

            trk.path[-1].bbox = [nx, ny, 1, 1]

    def add_linear_displacement(self, distance, angle):
        """
        Apply a linear displacement to offset a translation by a parent agent
        """
        if not self.has_active_tracks():
            return
        for i in range(len(self.active_tracks)):
            trk = self.active_tracks[i]

            last_d, last_v, delta_v, theta = trk.get_track_heading()
            new_posn = [last_d[0], last_d[1] + distance]

            trk.path[-1].bbox = [new_posn[0], new_posn[1], 1, 1]

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
            print()
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
        T = ObjectTrack(track_id, entity.class_id)
        T.add_new_step(entity, fc)
        self.global_track_store[track_id] = T
        self.active_tracks.append(T)

    def initialize_tracks(self, idx=0):
        """
        Address special case of initializing object tracks
        """
        self.active_tracks = collections.deque()
        curr_layer = self.layers[idx]
        for elem in curr_layer:
            self.create_new_track(elem, elem.class_id)

    def close_all_tracks(self):
        """
        Clear the active track queue in preparation for postprocessing
        """
        if self.active_tracks == None:
            return
        while len(self.active_tracks) > 0:
            self.inactive_tracks.append(self.active_tracks.pop())

    def link_all_tracks(self, min_len=0):
        """
        Resolve linked lists to make tracks externally traversible
        """
        link_counter = 0
        for k, v in self.global_track_store.items():
            if v.get_step_count() < min_len:
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
            pred.append((t.track_id, t.predict_next_box()))

        # create list of all pairs between track heads and detections in curr layer
        for c in range(len(curr_layer)):
            for p in pred:
                d = MathFxns.euclidean_dist(p[1], curr_layer[c].get_center_coord())
                pairs.append((p[0], c, d))

        # sort the list of pairs by euclidean distance
        sortkey = lambda s: s[2]
        pairs = sorted(pairs, key=sortkey)
        pc, tc, lc = 0, len(self.active_tracks), len(curr_layer)

        # update existing tracks with new entities
        while tc > 0 and lc > 0 and pc < len(pairs):
            elem = pairs[pc]
            if curr_layer[elem[1]].parent_track != None:
                pc += 1
                continue

            # Gate check for global nearest neighbors
            if elem[2] > ObjectTrackManager.constants["radial_exclusion"]:
                tc -= 1
                pc += 1
                continue

            # add entity to nearest track
            T = self.global_track_store[elem[0]]
            T.add_new_step(curr_layer[elem[1]], fc)

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
        fdict = None
        # construct "images" : []
        imgs = []
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
            # steps = []
            self.get_track(i).get_loco_track(fdict=None, steps=track_steps)
        for st in range(len(track_steps)):
            bbox = track_steps[st]["bbox"]
            x, y, w, h = bbox

            bbox[0], bbox[1] = x, y
            track_steps[st]["bbox"] = bbox

        for i in range(len(track_steps)):
            track_steps[i]["id"] = i
        return track_steps
