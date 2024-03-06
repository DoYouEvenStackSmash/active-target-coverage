#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

SLEEP_DURATION = 0.1  # How long do you want the environment to wait before proceeding?


def tracking_test(screen, environment, sample_rate=1):
    """
    A test where the agents in the environment track targets
    """
    flag = True
    counter = 0
    curr_pts = []
    pred_pts = []
    marked_pts = []

    # run until all targets have stopped moving
    while flag:
        flag = False
        pafn.clear_frame(screen)
        # allow agents to make predictions and move
        environment_agent_update(environment, True)

        if not counter % sample_rate:  # or not counter % 9:
            for i in range(len(environment.targets)):
                t = environment.targets[i]
                pafn.frame_draw_dot(screen, t.get_position(), t.color)
                marked_pts.append(t.get_position())
            # allow agents which cannot predict to move
            environment_agent_update(environment)
            # allow agents to see the targets
            environment.visible_targets()
        # render the environment
        environment_agent_illustration(
            screen, environment, sample_rate, curr_pts, pred_pts, marked_pts
        )

        # update the target positions

        for i in range(len(environment.targets)):
            t = environment.targets[i]
            flag = t.step() or flag

        pygame.display.update()
        time.sleep(SLEEP_DURATION)
        counter += 1

    # serialize the agent tracks for viewing
    for i in environment.agents:
        sensing_agent = environment.agents[i]
        import_agent_record(screen, sensing_agent.export_tracks())
    pygame.display.update()

    # wait for the user to click and exit
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1920, 1080)
    pafn.clear_frame(screen)

    # initialize agent 1

    sa = init_sensing_agent(_id=0, origin=(650, 50), width=np.pi, radius=400)
    sa.rotate_agent((500, 500))
    sa.set_tolerance(0.4)
    sa.track_lifespan=15
    sa._id = 0
    sa.heartbeat()
    sa.obj_tracker.avg_window_len = 4
    sa.ALLOW_PREDICTION = True
    # sa.ALLOW_TRANSLATION = False
    # sa.ALLOW_ROTATION = False

    ## uncomment to allow competing agent
    # initialize agent 2
    # sa2 = init_sensing_agent(_id=1, origin=(650,250), width=np.pi, radius=100)
    # sa2.rotate_agent((500,500))
    # sa2.heartbeat()
    # sa2._id = 1
    # sa2.ALLOW_PREDICTION = False

    sensing_agents = {}

    sensing_agents[sa._id] = sa
    # sensing_agents[sa2._id] = sa2

    targets = []
    # load json point files and initialize targets
    for i, file in enumerate(sys.argv[1:-1]):
        p = load_json_file(file)
        t = init_target(_id=i, path=p)
        targets.append(t)

    # initialize environment
    env = init_environment(sensing_agents=sensing_agents, targets=targets)

    measurement_rate = int(sys.argv[-1])

    tracking_test(screen, env, measurement_rate)


if __name__ == "__main__":
    main()
