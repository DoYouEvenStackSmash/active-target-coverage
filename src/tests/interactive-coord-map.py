#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

LALT = 256
LSHIFT = 1

def coord_map_sensing_agent(screen, sensing_agent):
    draw_sensing_agent(screen, sensing_agent)
    pygame.display.update()
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                pafn.clear_frame(screen)
                draw_sensing_agent(screen, sensing_agent)
                pt = pygame.mouse.get_pos()
                pafn.frame_draw_dot(screen, pt, pafn.colors["green"],0,6)
                det_pt = sensing_agent.transform_to_local_detection_coord(pt)
                sens_pt = sensing_agent.transform_to_local_sensor_coord((0,0),det_pt)
                print(f"orig:\t{pt}\n\tdet: {det_pt}\n\tsen: {sens_pt}")
                pygame.display.update()

def main():
    pygame.init()
    screen = pafn.create_display(600, 600)
    pafn.clear_frame(screen)
    origin = (300,300)
    sa = init_sensing_agent(origin=origin,width=3 *np.pi / 5,radius=200)
    coord_map_sensing_agent(screen, sa)

if __name__ == '__main__':
  main()
  