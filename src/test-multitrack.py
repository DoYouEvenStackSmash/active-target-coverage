#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment
from drawing_functions import *
from AnnotationLoader import AnnotationLoader as al
import pygame
import time
import sys

def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta

def import_agent_record(screen, agent_record):
    """
    Imports a LOCO formatted json

    Returns a json of some sort
    """

    fov_width = agent_record['sensor_params']['fov_width']
    fov_radius = agent_record['sensor_params']['fov_radius']

    trackmap = agent_record['trackmap']
    lt = agent_record['linked_tracks']

    get_center = lambda state: state['position']
    get_orientation = lambda state: state['orientation']
    annotations = agent_record['annotations']
    states = agent_record['states']
    # for anno in annotations:

    print(len(lt))
    for track in lt:
        pts = []
        color = None
        for step_id in track['steps']:
            anno = annotations[step_id]
            color = anno['track_color']
            state = states[anno['state_id']-1]
            x,y,w,h = anno['bbox']
            theta = (x - 50) / Sensor.WINDOW_WIDTH * fov_width
            theta = adjust_angle(get_orientation(state) + theta)
            r = y
            pt = mfn.pol2car(get_center(state), r, theta)
            pts.append(pt)
        
        render_path(screen, pts, color)
        pygame.display.update()
        # time.sleep(1)
    
    
def json_loader(filename):
    """
    LOADER
    legacy json loader
    Takes a filename and sys_path, opens a json file

    Returns a python dict
    """
    an_json = al.load_annotations_from_json_file(filename)
    if len(an_json) == 0:
        print("failed to load tracks")
        return None
    return an_json

def render_path(screen, path, color):
  for i in range(1, len(path)):
    # pafn.frame_draw_dot(screen, path[i], color)
    pafn.frame_draw_bold_line(screen, (path[i-1], path[i]), color)
    
def multitrack(screen, environment, paths,colors, n = 10):
  for k in environment.agents:
    draw_sensing_agent(screen, environment.agents[k])
  for i in range(n):
    for j,t in enumerate(environment.targets):
      t.origin = paths[j][i]
      pafn.frame_draw_dot(screen, paths[j][i], colors[j])
    
    for k in environment.agents:
      sensing_agent = environment.agents[k]
      if sensing_agent.obj_tracker.active_tracks == None:
        continue
      for t in range(len(sensing_agent.obj_tracker.active_tracks)):
        curr_pt, pred_pt = sensing_agent.estimate_next_detection(t)
        if len(pred_pt):
            # print((curr_pt,pred_pt))
            pafn.frame_draw_dot(screen, curr_pt, pafn.colors["red"])
            pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
            pafn.frame_draw_line(
                screen, (curr_pt, pred_pt), pafn.colors["white"]
            )

    environment.visible_targets()
    pygame.display.update()
    time.sleep(0.2)
  pafn.clear_frame(screen)
  for k in environment.agents:
    draw_sensing_agent(screen, environment.agents[k])
  for _id in environment.agents:
    sensing_agent = environment.agents[_id]
    e = sensing_agent.export_tracks()
    import_agent_record(screen, e)
    f = open(f"{_id}_out.json", "w")

    f.write(json.dumps(e, indent=2))
    f.close()
  time.sleep(5)
  sys.exit()

    # draw_sensing_agent(screen, environment.agents[k])
  

def path_handler(screen):
  min_x, min_y = 50,50
  max_x, max_y = 700,700
  # origins = [(100,50)]#, (50,50)]#, (50,100)]
  origins = [(100,50), (50,100)]
  origins.reverse()
  destinations = [(450,700), (700, 450), (700,700), (700,450)]
  vertices = []
  vertices.append(gfn.get_isosceles_vertex(origins[0], destinations[0],-1, 35))
  # vertices.append(gfn.get_isosceles_vertex(origins[1], destinations[1]))
  # vertices.append(gfn.get_midpoint(origins[1], destinations[1]))
  vertices.append(gfn.get_isosceles_vertex(origins[1], destinations[1],1,35))
  n = 30
  paths = []
  for i in range(len(origins)):
    l1 = gfn.lerp_list(origins[i], vertices[i], n)
    l2 = gfn.lerp_list(vertices[i], destinations[i], n)
    pts = []
    step = 1 / n
    for j in range(n):
      pts.append(gfn.lerp(l1[j], l2[j], step * j))
    pts.append(l2[-1])
    paths.append(pts)
  # paths[1].reverse()
  colors = [pafn.colors["green"], pafn.colors["red"], pafn.colors["cyan"]]
  environment = Environment()
  sensing_agent = init_sensing_agent()
  sensing_agent.ALLOW_TRANSLATION = False
  sensing_agent.ALLOW_ROTATION = False
  # theta, r = mfn.car2pol(origins[1], destinations[1])
  sensing_agent.centered_sensor.fov_radius = 900
  sensing_agent.centered_sensor.fov_width = np.pi / 2
  sensing_agent.exoskeleton.fov_theta = np.pi / 4

  for i in range(len(origins)):
    environment.add_target(Target(origins[i], _id=i))
  
  environment.agents[0] = sensing_agent

  for i,path in enumerate(paths):
    render_path(screen, path, colors[i])
  pygame.display.update()
  
  multitrack(screen, environment,paths, colors, n)
  
  time.sleep(10)
  sys.exit()

def init_sensing_agent(
    sensing_agent=SensingAgent(), origin=(0, 0), _id=0, orientation=(500, 500)
):
    ox, oy = origin
    scale = 2
    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
    # print(opts)

    mpt = gfn.get_midpoint(opts[0], opts[1])
    mpt2 = gfn.get_midpoint(mpt, opts[2])
    ap = Polygon(opts)
    rb = RigidBody(
        parent_agent=sensing_agent,
        ref_origin=mpt,
        ref_center=mpt2,
        endpoint=opts[2],
        rigid_link=ap,
    )
    sensor = Sensor(parent_agent=sensing_agent)
    sensor.fov_width = 3 * np.pi / 5

    sensing_agent.exoskeleton = rb
    sensing_agent.exoskeleton.states = []

    sensing_agent.centered_sensor = sensor
    sensing_agent.obj_tracker = ObjectTrackManager()
    sensing_agent.obj_tracker.linked_tracks = []
    sensing_agent.obj_tracker.layers = []
    sensing_agent.obj_tracker.trackmap = []
    sensing_agent.obj_tracker.global_track_store = {}

    sensing_agent.obj_tracker.parent_agent = sensing_agent
    sensing_agent._id = _id
    rotation = sensing_agent.rotate_agent(orientation)
    return sensing_agent

def main():
  pygame.init()
  screen = pafn.create_display(1000, 1000)
  pafn.clear_frame(screen)
  path_handler(screen)
  # environment = Environment()

if __name__ == '__main__':
  main()


  
