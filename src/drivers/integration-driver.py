#!/usr/bin/python3
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from support.render_support import TransformFxns as tfn
from Agent import *

from support.unit_norms import *
from support.Polygon import *

from support.World import *
from support.star_algorithm import *
from support.doubly_connected_edge_list import *

from PIL import Image, ImageDraw
import collections
from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from Scene import *
import json

from support.voronoi_regions import *
from support.feature_markers import *
from support.polygon_debugging import *
from support.region_tests import *
from support.file_loader import *
from support.transform_polygon import *

from support.Chain import Chain
from support.Link import Link

import pygame
import numpy as np
import sys
import time
COLLISION_THRESHOLD = 10
VERBOSE = True
SAMPLE_RATE = 400
LALT = 256
LSHIFT = 1
LCTRL = 64
SPACE = 32
OFFT = 20
SPLINE_COUNT = 2
TRANSLATE = False


def draw_all_normals(screen, chain):
  '''
  Render function for all coordinate frames in a chain
  Does not return
  '''
  nl = chain.get_chain_normals()
  points = chain.get_chain_point_sets()
  for l in chain.links:
    n = l.get_normals()
    pafn.frame_draw_line(screen, (l.get_origin(), n[0]), pafn.colors['tangerine'])
    pafn.frame_draw_line(screen, (l.get_origin(), n[1]), pafn.colors['yellow'])
  pafn.frame_draw_dot(screen, chain.get_anchor_origin(), pafn.colors["cyan"])
  # pafn.frame_draw_dot(screen, chain.links[2].get_endpoint(), pafn.colors["cyan"])


def draw_all_links(screen, chain):
  '''
  Render function for all polygon links in the chain
  Does not return
  '''
  points = chain.get_chain_point_sets()
  for p in range(len(points)):
    pafn.frame_draw_polygon(screen, points[p], pafn.colors["red"])


def translate_chain(screen, chain, target_point, steps=30):
  '''
  Translates a chain of links to a target point
  '''
  r,theta = get_polar_coord(chain.get_anchor_origin(), target_point)
  step_r = r / steps
  x_step = step_r * np.cos(theta)
  y_step = step_r * np.sin(theta)
  total_x, total_y = 0,0
  for i in range(steps):
    total_x+=x_step
    total_y+=y_step
    for link in chain.links:
      
      link.translate_body(x_step, y_step)
      if steps > 1:
        pafn.clear_frame(screen)
        draw_all_normals(screen, chain)
        draw_all_links(screen, chain)
        pygame.display.update()
        time.sleep(0.005)
  ax, ay = chain.get_anchor_origin()
  chain.origin = (ax + total_x, ay + total_y)
  # chain.anchor_origin = chain.origin
  
def draw_all_normals(screen, chain):
  '''
  Render function for all coordinate frames in a chain
  Does not return
  '''
  nl = chain.get_chain_normals()
  points = chain.get_chain_point_sets()
  for l in chain.links:
    n = l.get_normals()
    pafn.frame_draw_line(screen, (l.get_origin(), n[0]), pafn.colors['tangerine'])
    pafn.frame_draw_line(screen, (l.get_origin(), n[1]), pafn.colors['yellow'])
  pafn.frame_draw_dot(screen, chain.get_anchor_origin(), pafn.colors["cyan"])
  pafn.frame_draw_dot(screen, chain.links[-1].get_endpoint(), pafn.colors["cyan"])


def draw_all_links(screen, chain):
  '''
  Render function for all polygon links in the chain
  Does not return
  '''
  points = chain.get_chain_point_sets()
  for p in range(1,len(points)):
    pafn.frame_draw_polygon(screen, points[p], pafn.colors["red"])

def calculate_circles(screen, chain,target_point,DRAW=None):
  '''
  Calculates intersection point of outermost circle
  https://mathworld.wolfram.com/Circle-CircleIntersection.html
  
  Returns a list of 3 points, which describe the chord going through the lens.
  '''
  t_x,t_y = target_point
  rad,inner_len = mfn.car2pol(chain.links[1].get_origin(), chain.links[1].get_endpoint())
  rad2,outer_len = mfn.car2pol(chain.links[-1].get_origin(), chain.links[-1].get_endpoint())

  o_x,o_y = chain.get_anchor_origin()

  target_distance = np.sqrt(np.square(t_x - o_x) + np.square(t_y - o_y))
  x = np.divide((np.square(inner_len) + np.square(target_distance) - np.square(outer_len)), (2 * inner_len))
  
  y = np.sqrt(np.square(target_distance) - (np.divide(np.square(np.square(inner_len) + np.square(target_distance) - np.square(outer_len)), (4 * np.square(target_distance)))))
  max_radius = abs(outer_len)
  curr_radius = abs(outer_len - x)
  y = min(np.sqrt(abs(np.square(max_radius) - np.square(curr_radius))), y)
  ps = [(o_x + x, o_y + y), (o_x + x, o_y), (o_x + x, o_y - y)]
  if DRAW == None:
    return ps

  # # pps = transform_point_set(link, ps)
  pafn.clear_frame(screen)
  for i in ps:
    pafn.frame_draw_dot(screen, i, pafn.colors["magenta"])
  # return pps
  
  pafn.draw_circle(screen, target_distance, (o_x, o_y), pafn.colors["yellow"])
  pafn.draw_circle(screen, outer_len, chain.links[2].get_origin(), pafn.colors["cyan"])
  pafn.draw_circle(screen, x, (o_x, o_y), pafn.colors["green"])
  pygame.display.update()
  return ps

def draw_bundle(screen, chain, Olist = [], A = None):
  '''
  Bundles all drawing functions together
  Does not return
  '''
  for O in Olist:
    sanity_check_polygon(screen, O)
  if A != None:
    sanity_check_polygon(screen, A)
  draw_all_normals(screen, chain)
  draw_all_links(screen, chain)

def rotate_two_link_chain(screen, chain, target_point, intermediate_point, steps = 30, Olist = [], A=None, VERBOSE = False):
  '''
  Rotates links in the chain as influenced by a target point
  Does not return
  Calls update
  '''
  # rotate last link to circle intersection
  rad2 = chain.links[-1].get_relative_rotation(intermediate_point)
  # rotate first link from circle intersection to target
  rad1,tp = tfn.calculate_rotation(chain.get_anchor_origin(), target_point, intermediate_point)
  # calculate rotation matrices by number of steps
  rot_mat1 = tfn.calculate_rotation_matrix(rad1,step_count=steps)
  rot_mat2 = tfn.calculate_rotation_matrix(rad2,step_count=steps)
  step = np.divide(rad1, steps)
  step2 = np.divide(rad2, steps)
  origin = chain.links[0].get_origin()
  '''
    For each step
      for each link
        check collision against each obstacle
        If none, rotate
        else return
      for last link
        rotate by remainder
  '''
  
  v = 0
  for i in range(steps):
    for j in range(1,len(chain.links)):
      l = chain.links[j]    
      for A in Olist:
        v = check_contact(screen, l.get_body(), A, VERBOSE)
        if v < COLLISION_THRESHOLD:
          return v
      l.rotate_body(origin, rot_mat1)
      l.rel_theta += step
    
    for A in Olist:
      v = check_contact(screen, l.get_body(), A, VERBOSE)
      if v < COLLISION_THRESHOLD:
        return v
      # continue
    chain.links[-1].rotate_body(chain.links[-1].get_origin(), rot_mat2)
    chain.links[-1].rel_theta += step2
    
    if steps > 1:
      pafn.clear_frame(screen)
      draw_bundle(screen, chain,Olist = Olist, A = A)
      pygame.display.update()
  return 0


def check_contact(screen, A, O, VERBOSE = False):
  '''
  Collision detection wrapper for an Agent and an obstacle
  Returns the distance between closest pair of points on agent and obstacle
  '''
  val = find_contact(build_star(A.get_front_edge(), O.get_front_edge()), screen, VERBOSE)
  
  # if collision, draw boundary region(minkowski sum)
  if val < COLLISION_THRESHOLD:
    obs_spc = construct_star_diagram(A, O)
    pafn.frame_draw_polygon(screen, obs_spc, pafn.colors['yellow'])
    pygame.display.update()
  return val

def render_point_segments(screen, pts, p):
  '''
  Helper function for rendering segments
  Does not return
  '''
  # draw lines for points existing thus far
  for i in range(1,min(len(pts),4)):
    pafn.frame_draw_line(screen, (pts[i-1],pts[i]), pafn.colors['green'])
  pafn.frame_draw_dot(screen, p, pafn.colors['green'])

  # draw lines for next set of points to aid in guidance
  if len(pts) > 4:
    for j in range(4, len(pts)):
      pafn.frame_draw_line(screen, (pts[j-1],pts[j]), pafn.colors['tangerine'])
    pafn.frame_draw_dot(screen, p, pafn.colors['tangerine'])

def min_pt(p1, p2, t):
  '''
  Computes closest point to target
  Returns a point
  '''

  r1_dist, r1_theta = get_polar_coord(p1,t)
  r2_dist, r2_theta = get_polar_coord(p2,t)
  mpt = p1
  if abs(r1_dist) > abs(r2_dist):
    mpt = p2
  return mpt

def preprocess_circles(screen, chain, p):
  '''
  Preprocesses circle circle intersection for a chain and a target point
  Returns a point
  '''
  ps = calculate_circles(screen, chain, p)
  r,t = tfn.calculate_rotation(chain.get_anchor_origin(), chain.links[1].get_endpoint(), ps[1])
  print(f"rotation amount: {r}")
  # if r == 0 and i != 0:
  #   print("skipping")
  #   continue
  rot_mat = tfn.calculate_rotation_matrix(r,step_count=1)
  ps = tfn.rotate_point_set(chain.get_anchor_origin(), ps, rot_mat)
  mpt1 = min_pt(ps[0],ps[2],p)
  mpt2 = min_pt(ps[1],mpt1,p)
  return mpt2#min_pt(mpt1,mpt2,p)

def predict_coverage(A,screen):
  estimates = A.predict_targets_covered()
  for i in estimates:
    pred = A.transform_from_local_coord(i[1][0],i[1][1])
    curr = A.transform_from_local_coord(i[0][0],i[0][1])
    if len(pred) > 0:
      if screen != None:
        pafn.frame_draw_dot(screen, curr, pafn.colors['red'])
        pafn.frame_draw_line(screen, (curr,pred), pafn.colors["white"])

def pygame_chain_move(screen, chain, A = None, T = None ,Olist = []):
  origin = (400,400)
  T.origin = (550,550)
  Tlist = [T]
  Alist = [A]
  
  '''
  Driver function interactions between two polygons A and static O
  Mouse driven path following
  '''
  global COLLISION_THRESHOLD
  # pts = []
  mod = 0
  pts = [chain.links[i].get_origin() for i in range(len(chain.links))]
  pts.append(chain.links[-1].get_endpoint())
  destinations = []
  step_size = 15
  
  for i in range(20):
    x,y = T.origin
    # destinations.append((x, y - step_size * i))
    destinations.append(((x - step_size * i), y))
  destinations.reverse()
  for i in reversed(destinations):
    destinations.append(i)
  for t in range(len([Tlist])):
    ptr = Tlist[t].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  pygame.display.update()
    # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
    # Tlist[t].origin = pt
  translation_path = destinations
  for k in range(1,len(translation_path)):

    pt = translation_path[k]
    flag = 3
    print("process")
    while flag != Agent.VALID:
      if k == len(translation_path) - 1:
        flag = Agent.VALID
        return
      v = 0
      
      p,flag = A.publish_adjustment()
      print(flag)
      if len(p) and flag != Agent.VALID:

        tp2 = preprocess_circles(screen, chain, p)

        tp1 = p
        pafn.clear_frame(screen)
        if flag == Agent.ANGULAR:
          v = rotate_two_link_chain(screen, chain, tp1,tp2, steps=1,Olist = Olist, A = A, VERBOSE = True)
          A.rotate_sensor(chain.links[-1].get_endpoint())
          # draw_bundle(screen, chain)
          # draw_coordinate_frame(screen, A)
        # draw_bundle(screen, chain,Olist=Olist)
        # draw_coordinate_frame(screen, A)  
        # continue
        for t in range(len([Tlist])):
          # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
          Tlist[t].origin = pt
          ptr = Tlist[t].get_origin()
          pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])

          # COLLISION_THRESHOLD = 1
          # return
        if flag == Agent.RANGE:
          disp = A.translate_sensor(p)
          npt = mfn.pol2car(chain.links[-1].get_origin(), disp, chain.links[-1].get_relative_angle())
          v = translate_chain(screen, chain, npt, 1)
          if v == None and v > 0:
            draw_coordinate_frame(screen, A)
            draw_bundle(screen, chain)
            pygame.display.update()
            return
          # theta, r = 
        # if v != None and v > 1:
        #   return
        # v = check_contact(screen, A=chain.links[-1], O=Olist)
        draw_bundle(screen, chain)
        draw_coordinate_frame(screen, A)
        # break        
        # for j in range(i,len(destinations)):
        #   pafn.frame_draw_dot(screen, destinations[j], pafn.colors["cyan"])
        pygame.display.update()
      # else:
      #   flag = Agent.VALID
      time.sleep(0.1)
    theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
    for t in range(len([Tlist])):
      # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
      Tlist[t].origin = pt
      ptr = Tlist[t].get_origin()
      pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
    for a in range(len(Alist)):
      visible_targets(Alist[a], Tlist)
    # pygame.display.update()

    time.sleep(0.01)
    pygame.display.update()
        # COLLISION_THRESHOLD = 10
        # return
  e = A.export_tracks()
  f = open("out.json", "w")
  f.write(json.dumps(e, indent = 2))
  f.close()


def repeatable_test(screen, A, T):
  Tlist = [T]
  Alist = [A]
  T.origin = (500,500)
  step_size = 30
  destinations = []
  origin = (500,500)
  for i in range(25):
    x,y = T.origin
    # destinations.append((x, y - step_size * i))
    destinations.append(((x - step_size * i), y))
  destinations.reverse()
  for i in reversed(destinations):
    destinations.append(i)
  for t in range(len([Tlist])):
    # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
    # Tlist[t].origin = pt
    ptr = Tlist[t].get_origin()
    pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
  
  
  translation_path = destinations
  print(destinations)
  print("running\n")

  rotate_counter = 0
  for pt in translation_path[1:]:
    
    pafn.clear_frame(screen)
    # T.origin = pt
    for a in range(len(Alist)):
      A = Alist[a]
      predict_coverage(A,screen)
      A.adjust_for_coverage()
      # adjust_for_coverage(A,Tlist,screen)
    # time.sleep(.05)
      draw_coordinate_frame(screen, A)
    pygame.display.update()
    time.sleep(.1)
    theta, r = mfn.car2pol(Tlist[0].get_origin(), pt)
    for t in range(len([Tlist])):
      # Tlist[t].origin = mfn.pol2car(Tlist[t].get_origin(), r, theta)
      Tlist[t].origin = pt
      ptr = Tlist[t].get_origin()
      pafn.frame_draw_dot(screen, ptr, pafn.colors["green"])
    pygame.display.update()
    for a in range(len(Alist)):
      visible_targets(Alist[a], Tlist)
    time.sleep(0.2)
    
  
  print(f"num_steps:{len(translation_path)}")
  print(f"rotations:{rotate_counter}")
  
  # for A in Alist:
  e = A.export_tracks()
  f = open("out.json", "w")
  f.write(json.dumps(e, indent = 2))
  f.close()
      # pygame.display.update()


def init_chain(screen):
  # pts = [(350, 390),(450,390),(450,410),(350,410)]
  pts = [(350, 390),(420,400),(350,410)]
  pts2 = [(470, 390),(550,390),(550,410), (470,410)]
  origin = (400,400)
  opts = [origin, (351,401), (352,402)]
  ap = Polygon(opts)
  ap.color = pafn.colors["green"]
  ap.v_color = pafn.colors["cyan"]
  ap.e_color = pafn.colors["tangerine"]
  a = Link(endpoint = origin, rigid_body = ap)
  # a.theta = np.pi / 2
  A = Polygon(pts)
  
  # A.theta = 0
  B = Polygon(pts2)
  A.color = pafn.colors["green"]
  A.v_color = pafn.colors["cyan"]
  A.e_color = pafn.colors["tangerine"]
  Olist = []
  # sanity_check_polygon(screen,A)
  
  c = Chain(origin = origin, anchor=a)
  l = Link(endpoint=(410,400), rigid_body = A)
  p = (400, 500)
  

  tp1 = p
  pafn.clear_frame(screen)
  # Olist = []
  
  B.color = pafn.colors["green"]
  B.v_color = pafn.colors["cyan"]
  B.e_color = pafn.colors["tangerine"]
  # B = build_polygon(sys.argv[1])
  for arg in sys.argv[1:]:
    o = build_polygon(arg)
    o.color = pafn.colors["white"]
    o.v_color = pafn.colors["cyan"]
    o.e_color = pafn.colors["tangerine"]
    Olist.append(o)
  
  e = Link(endpoint = (600,400), rigid_body = B)
  c.add_link(l)
  tp2 = preprocess_circles(screen, c, p)
  v = rotate_two_link_chain(screen, c, tp1,tp2, steps=1,VERBOSE = True)
  # c.add_link(e)
  draw_bundle(screen, c, Olist = Olist)
  # draw_all_normals(screen, c)
  # draw_all_links(screen, c)
  
  pygame.display.update()
  # start pygame loop
  return c, Olist

def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  pygame.display.update()
  c,o = init_chain(screen)
  A = Agent([400,400], [c.links[-1].get_relative_angle(), 200, np.pi / 4], obj_tracker = ObjectTrackManager())
  
  T = Target((500,550))
  pygame_chain_move(screen, c, A, T, Olist = o)
  # repeatable_test(screen, A, T)#,T2])

if __name__ =='__main__':
  main()
  