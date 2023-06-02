#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *
AGENT_MAX_X = 100


def convert_to_sensor_coordinate(x, y, z, max_x, max_z, sensor_fov_width):
  """
  x = [0,1]
  y = [0,1]
  max_x = width of frame
  max_y = height of frame
  """
  # get x normalized to be in (0,100)
  rel_x = (x * max_x - (max_x / 2)) / max_x * AGENT_MAX_X + AGENT_MAX_X / 2
  rel_z = (z * max_z - (max_z / 2)) / max_z * AGENT_MAX_X + AGENT_MAX_X / 2
  rel_y = 100
  # rel_z = max_z * z


  bbox = [rel_x, rel_y]

  # get angle theta to be a displacement from agent zero
  theta = (rel_x / 100) * sensor_fov_width - (sensor_fov_width / 2)
  phi = (rel_z / 100) * sensor_fov_width - (sensor_fov_width / 2)


  det_posn = mfn.pol2car((0,0,0), rel_y, theta, phi)
  return det_posn



def sensor_pov_coordinate(screen):
  max_x = 600
  max_y = 600
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.MOUSEBUTTONDOWN:
        while pygame.MOUSEBUTTONUP not in [
            event.type for event in pygame.event.get()
        ]:
            continue
        p = pygame.mouse.get_pos()
        x,y = p
        x = x / max_x
        z = y / max_y
        y = y / max_y
        # z = y / max_y
        # y = max_y - y

        print(convert_to_sensor_coordinate(x, y, z, 600, 600, 3*np.pi / 5))

          

def main():
    pygame.init()
    screen = pafn.create_display(600, 600)
    pafn.clear_frame(screen)

    origin = (300,300, 0)

    pafn.frame_draw_dot(screen, origin, pafn.colors["green"], 0, 1)
    pafn.frame_draw_cross(screen, origin, pafn.colors["green"])
    sensor_pov_coordinate(screen)

if __name__ == '__main__':
  main()