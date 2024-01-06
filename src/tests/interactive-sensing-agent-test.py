#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *


def active_mouse_test(screen, environment, measurement_rate=1):
    """
    A test for tracking the user's mouse according to some measurement rate
    """
    pt = None
    last_pt = None
    counter = 0
    curr_pts = []
    pred_pts = []
    marked_pts = []
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()

            pt = pygame.mouse.get_pos()

            last_pt = pt

            pafn.clear_frame(screen)
            environment_agent_update(environment, True)

            environment.targets[0].origin = pt
            counter += 1
            if not counter % measurement_rate:
                marked_pts.append(pt)
                environment.visible_targets()
            environment_agent_illustration(
                screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts
            )
        pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1400, 1000)
    pafn.clear_frame(screen)

    # initialize agent
    sa = init_sensing_agent(origin=(50, 400), width=np.pi / 2, radius=200)
    sa.track_lifespan=15
    
    sa.heartbeat()
    sa.obj_tracker.avg_window_len = 2

    sensing_agents = {}
    sensing_agents[sa._id] = sa
    t = init_target()
    env = init_environment(sensing_agents=sensing_agents, targets=[t])
    md = 1
    if len(sys.argv) > 1:
        md = int(sys.argv[-1])
    active_mouse_test(screen, env, md)


if __name__ == "__main__":
    main()
