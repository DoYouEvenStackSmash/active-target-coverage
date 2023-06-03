#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *


def tracking_test(screen, environment):
    flag = True
    while flag:
        flag = False
        pafn.clear_frame(screen)

        for k in environment.agents:
            sensing_agent = environment.agents[k]

            r, t = sensing_agent.tracker_query()
            sensing_agent.reposition(r, t)
            sensing_agent.heartbeat()

            render_predictions(screen, sensing_agent)
            draw_sensing_agent(screen, sensing_agent)

        for i in range(len(environment.targets)):
            t = environment.targets[i]
            pafn.frame_draw_dot(screen, t.get_position(), t.color)
            flag = t.step() or flag

        pygame.display.update()
        time.sleep(0.05)
        environment.visible_targets()
    for i in environment.agents:
        sensing_agent = environment.agents[i]
        import_agent_record(screen, sensing_agent.export_tracks())
    pygame.display.update()

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    sensing_agents = {}
    sa = init_sensing_agent(origin=(500, 200, 0), radius=300)
    sensing_agents[sa._id] = sa
    targets = []
    # load json point files
    for i, file in enumerate(sys.argv[1:]):
        p = load_json_file(file)
        t = init_target(_id=i, path=p)
        targets.append(t)
    env = init_environment(sensing_agents=sensing_agents, targets=targets)
    time.sleep(0.2)
    tracking_test(screen, env)


if __name__ == "__main__":
    main()
