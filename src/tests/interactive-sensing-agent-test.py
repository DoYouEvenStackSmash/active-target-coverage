#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from sim_env_init import *

from drawing_functions import *


def mouse_test(screen, environment):
    pt = None
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()

            pt = pygame.mouse.get_pos()
            if pt == last_pt:
                continue
            last_pt = pt

            pafn.clear_frame(screen)
            pafn.frame_draw_dot(screen, pt, pafn.colors["green"], 0, 6)

            for k in environment.agents:
                sensing_agent = environment.agents[k]

                # r, t = sensing_agent.tracker_query()
                # print(f"estimated rotation {r}")
                # sensing_agent.reposition(r, t)
                sensing_agent.heartbeat()
                arr = sensing_agent.estimate_next_detection()
                draw_sensing_agent(screen, sensing_agent)
                if not len(arr):
                    continue
                for i in range(len(arr)):
                    curr_pt, pred_pt = arr[i]
                    cppt = sensing_agent.transform_from_frame_coord(curr_pt)
                    ppt = sensing_agent.transform_from_frame_coord(pred_pt)
                    predpt = environment.convert_from_agent_coordinates(0, ppt)
                    currpt = environment.convert_from_agent_coordinates(0, cppt)
                    crpt = gfn.reduce_dimension(currpt.get_cartesian_coordinates())
                    prpt = gfn.reduce_dimension(predpt.get_cartesian_coordinates())
                    pafn.frame_draw_dot(screen, crpt, pafn.colors["tangerine"])
                    pafn.frame_draw_dot(screen, prpt, pafn.colors["yellow"])
                    pafn.frame_draw_line(screen, [crpt,prpt], pafn.colors["white"])
                # render_predictions(screen, sensing_agent)

                
            pygame.display.update()
            p = Position()
            p.set_by_triple(gfn.raise_dimension(pt))
            environment.targets[0].origin = p
            environment.visible_targets()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    sensing_agents = {}
    sa = init_sensing_agent(origin=Position(400, 400, 0))
    sensing_agents[sa._id] = sa
    t = init_target()
    print(t.get_origin())
    env = init_sim_environment(sensing_agents=sensing_agents, targets=[t])
    mouse_test(screen, env)


if __name__ == "__main__":
    main()
