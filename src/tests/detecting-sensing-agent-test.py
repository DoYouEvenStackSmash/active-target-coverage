#!/usr/bin/python3
"""
A test for sanity checking an agent's detections vs visibility
"""
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

LALT = 256
LSHIFT = 1


def grid_sensing_agent(screen, sensing_agent, points):
    for p in points:
        # p = pygame.mouse.get_pos()
        visible = sensing_agent.is_visible(p)
        if visible:
            pafn.frame_draw_dot(screen, p, pafn.colors["green"], 8, 6)
        else:
            pafn.frame_draw_dot(screen, p, pafn.colors["red"], 8, 6)
        pt = sensing_agent.transform_to_local_detection_coord(p)

        pt2 = sensing_agent.transform_to_local_sensor_coord((0, 0), pt)
        pt3 = sensing_agent.transform_to_global_coord(pt)

        detectable, flag = sensing_agent.is_detectable(pt2)

        if detectable:
            pafn.frame_draw_dot(screen, p, pafn.colors["cyan"])
        else:
            pafn.frame_draw_dot(screen, p, pafn.colors["red"])
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
                    rotation = sensing_agent.rotate_agent(p)

                for p in points:
                    # p = pygame.mouse.get_pos()
                    visible = sensing_agent.is_visible(p)
                    if visible:
                        pafn.frame_draw_dot(screen, p, pafn.colors["green"], 8, 6)
                    else:
                        pafn.frame_draw_dot(screen, p, pafn.colors["red"], 8, 6)
                    pt = sensing_agent.transform_to_local_detection_coord(p)

                    pt2 = sensing_agent.transform_to_local_sensor_coord((0, 0), pt)
                    pt3 = sensing_agent.transform_to_global_coord(pt)

                    detectable, flag = sensing_agent.is_detectable(pt2)

                    if detectable:
                        pafn.frame_draw_dot(screen, p, pafn.colors["cyan"])
                    else:
                        pafn.frame_draw_dot(screen, p, pafn.colors["red"])
                draw_sensing_agent(screen, sensing_agent)
                pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    origin = (500, 500)
    sa = init_sensing_agent(origin=origin)
    p = load_json_file(sys.argv[1])
    grid_sensing_agent(screen, sa, p)


if __name__ == "__main__":
    main()
