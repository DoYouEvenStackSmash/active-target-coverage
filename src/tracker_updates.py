def visibility(Adict, Tlist):
  pairs = []
  sortkey = lambda x: x[2]
  for k,A in Adict.items:
    for target in Tlist:
      d = mfn.euclidean_dist(A.origin, target.get_origin())
      pairs.append((A, target, d))
  pairs = sorted(pairs, key=sortkey)
  c = 0
  pl = []
  while c < len(pairs):
    if pairs[c][2] > pairs[c][0].fov_radius:
      break
    if pairs[c][0].is_visible(pairs[c][1].get_origin()):
      # pairs[c][0].add_detection(pairs[c][1].get_origin())
      print(f"{pairs[c][0]._id} has f{pairs[c][1]}")
      pl.append((pairs[c][0], pairs[c][1]))
    c+=1
  return pl


def visible_targets(A, Tlist):
  A.obj_tracker.init_new_layer()
  frame_id = "frame_"+str(len(A.obj_tracker.layers))
  for target in Tlist:
    d = mfn.euclidean_dist(A.origin, target.get_origin())
    pairs.append((A, target, d))
  pairs = sorted(pairs, key=sortkey)
  c = 0
  pl = []
  while c < len(pairs):
    if pairs[c][2] > pairs[c][0].fov_radius:
      break
    if pairs[c][0].is_visible(pairs[c][1].get_origin()):
      dc = A.compute_detection_coords(T.get_origin())
      yb = sann.register_annotation(0, dc, frame_id)
      A.obj_tracker.add_new_element_to_layer(yb)
  A.process_layer(len(A.obj_tracker.layers) -1)

def add_visible_to_tracker(pairlist, frame_id = 0):
  new_layers_added = set()
  for A, T in pairlist:
    if A not in new_layers_added:
      A.init_new_layer()
      new_layers_added.add(A)
    dc = A.compute_detection_coords(T.get_origin())
    yb = sann.register_annotation(0, dc, f"frame_{frame_id}")
    A.obj_tracker.add_new_element_to_layer(yb)
  for 

