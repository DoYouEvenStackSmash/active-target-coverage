#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

LALT = 256
LSHIFT = 1

def grid_sensing_agent(screen, sensing_agent, points):
    draw_sensing_agent(screen, sensing_agent)
    pygame.display.update()
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == LALT:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    
                    pafn.clear_frame(screen)
                    sensing_agent.translate_agent(p)
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    pafn.clear_frame(screen)
                    p = Position(p[0], p[1])
                    rotation = sensing_agent.rotate_agent(p)

                for p in points:
                    # p = pygame.mouse.get_pos()
                    
                    visible = sensing_agent.is_visible(p)
                    
                    rpt = gfn.reduce_dimension(p.get_cartesian_coordinates())
                    if visible:
                        pafn.frame_draw_dot(screen, rpt, pafn.colors["green"], 8, 6)
                    else:
                        pafn.frame_draw_dot(screen, rpt, pafn.colors["red"], 8, 6)                    
                    pt = sensing_agent.transform_to_local_detection_coord(p)

                    pt2 = sensing_agent.transform_to_local_sensor_coord(Position(0, 0, 0), pt)
                    pt3 = sensing_agent.transform_to_global_coord(pt)

                    detectable, flag = sensing_agent.is_detectable(pt2)

                    if detectable:
                        pafn.frame_draw_dot(screen, rpt, pafn.colors["cyan"])
                    else:
                        pafn.frame_draw_dot(screen, rpt, pafn.colors["red"])
                draw_sensing_agent(screen, sensing_agent)
                pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    origin = Position(500,500, 0)
    sa = init_sensing_agent(origin=origin)
    p = load_json_file(sys.argv[1])
    for i in range(len(p)):
        print(p[i])
        p[i] = Position(p[i][0], p[i][1], p[i][2])
    grid_sensing_agent(screen, sa, p)

if __name__ == '__main__':
    main()