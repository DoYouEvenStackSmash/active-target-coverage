#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *


def static_mouse_test(screen, environment, measurement_rate=1):
    """
    A test for tracking the user's mouse at a certain measurement rate
    """
    pt = None
    last_pt = (0, 0)
    pred_pts = []
    curr_pts = []
    marked_pts = []
    counter = 0

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()

            pt = pygame.mouse.get_pos()

            pafn.clear_frame(screen)

            # update the agents
            environment_agent_update(environment, True)

            environment.targets[0].origin = pt
            counter += 1
            if not counter % measurement_rate:
                last_pt = pt
                if len(curr_pts):
                    marked_pts.append(curr_pts[-1])
                # update any agents which are not allowed to predict
                environment_agent_update(environment)
                # allow the agents to see the target
                environment.visible_targets()

            # render all the agents, targets, and paths
            environment_agent_illustration(
                screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts
            )
            pafn.frame_draw_dot(screen, last_pt, pafn.colors["green"], 4, 8)
            pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)

    sensing_agents = {}
    sa = init_sensing_agent(_id=1, origin=(480, 900), width=np.pi, radius=7000)
    sa.rotate_agent((500, 500))
    # freeze the agent
    sa.ALLOW_ROTATION = False
    sa.ALLOW_TRANSLATION = False
    sa._id = 1
    sa.exoskeleton.color = pafn.colors["yellow"]
    sa.heartbeat()

    sensing_agents[sa._id] = sa
    t = init_target()
    env = init_environment(sensing_agents=sensing_agents, targets=[t])

    md = 1
    if len(sys.argv) > 1:
        md = int(sys.argv[-1])
    static_mouse_test(screen, env, md)


if __name__ == "__main__":
    main()
