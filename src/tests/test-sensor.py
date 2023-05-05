#!/usr/bin/python3
from Sensor import *
import pygame
import time
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
  repeatable_test(screen, s)

if __name__ == '__main__':
  main()
  

