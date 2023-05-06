#!/usr/bin/python3
from Sensor import *
import pygame
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

def draw_coordinate_frame(screen, S):
  '''
  Helper function for displaying the curved coordinate fov of agent A
  Does not return
  '''
  levels = 5
  coord_frame = S.get_visible_fov(levels)
  detect_frame = S.get_detectable_bounds(levels)
  for c in range(levels):
    for i in range(1,len(coord_frame[c])):
      pafn.frame_draw_line(screen, (coord_frame[c][i-1], coord_frame[c][i]), pafn.colors['white'])
      
  for endpoint in coord_frame[-1]:
    pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors['white'])
  
  for endpoint in detect_frame[-1]:
    pafn.frame_draw_line(screen, (S.origin, endpoint), pafn.colors['tangerine'])


TIME = .5

def interactive_sensor_test(screen, S):
  draw_coordinate_frame(screen, S)
  pygame.display.update()

  dest = None
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        # LCTRL for exit hotkey
        print(S.fov_theta)
        
        # Export the tracks
        if pygame.key.get_mods() == LCTRL:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          theta, r = mfn.car2pol(S.origin, p)
          S.adjust_orientation(theta)
          draw_coordinate_frame(screen, S)
          pygame.display.update()
          continue
        # rotate the sensor
        elif pygame.key.get_mods() == LSHIFT:  # rotate relative
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pt = S.transform_to_local_coord(p)
          print(pt)
          ratio = pt[0] / 100
          theta = 0
          # if 0 < ratio and ratio < 1:
          theta = (0.5 - ratio) * S.fov_width
          
          if theta > np.pi:
            theta = -2 * np.pi + theta
          elif theta < -np.pi:
            theta = 2 * np.pi + theta 
          
          mod_theta = S.fov_theta
          if mod_theta + theta > np.pi:
            mod_theta = mod_theta + theta + -2 * np.pi
          elif mod_theta + theta < -np.pi:
            mod_theta = mod_theta + theta + 2 * np.pi
          else:
            mod_theta = mod_theta + theta

          print(f"{theta} vs {mod_theta}")
          S.adjust_orientation(mod_theta)

          
          # print(init_theta)
          # print(pt)
          
          # print(theta)
          # print(theta / S.fov_width)
          # S.adjust_orientation(init_theta)

          draw_coordinate_frame(screen, S)
          pygame.display.update()
          
          continue

        # estimate the next position in the track
        elif pygame.key.get_mods() == LALT: # estimate
          pafn.clear_frame(screen)
          pygame.display.update()
          continue

        else:
          while pygame.MOUSEBUTTONUP not in [event.type for event in pygame.event.get()]:
            continue
          p = pygame.mouse.get_pos()
          pt = S.transform_to_local_coord(p)
          # print(pt)
          dc = S.transform_from_local_coord(pt[0],pt[1])
          print(f"orig {p}\tflop {dc}\nintermediate {pt}")
          draw_coordinate_frame(screen, S)

def repeatable_sensor_test(screen, S):
  directions = [-np.pi, -np.pi / 2, 0,  np.pi / 2]
  target_points = [(450,450), (550, 450), (550,550), (450,550)]
  origins = [(450, 500), (500, 450), (550, 500), (500,550)]
  
  for pt in origins:
    # theta, r = mfn.car2pol(S.origin, pt)
    pafn.clear_frame(screen)
    for fov in directions:
      S.adjust_orientation(fov)
      draw_coordinate_frame(screen, S)
      for tpt in target_points:
        
        status = S.is_visible(tpt)
        
        if status:
          status2 = S.is_detectable(S.transform_to_local_coord(tpt))
          if status2:
            pafn.frame_draw_dot(screen, tpt, pafn.colors["cyan"])
          print(S.transform_to_local_coord(tpt))
        else:
          pafn.frame_draw_dot(screen, tpt, pafn.colors["red"])

      pygame.display.update()
      time.sleep(TIME)
    S.origin = pt



def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  s = Sensor()
  s.fov_width = 3 * np.pi / 5
  s.origin = (500,500)
  interactive_sensor_test(screen, s)
  # repeatable_test(screen, s)

if __name__ == '__main__':
  main()
  

