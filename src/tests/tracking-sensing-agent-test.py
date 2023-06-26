#!/usr/bin/python3
"""
Legacy test which allows the agent to see the target at every time step
"""
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

SLEEP_DURATION = 0.1


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

            # render the prediction of the agent
            render_predictions(screen, sensing_agent)
            draw_sensing_agent(screen, sensing_agent)

        for i in range(len(environment.targets)):
            t = environment.targets[i]
            pafn.frame_draw_dot(screen, t.get_position(), t.color)
            flag = t.step() or flag

        pygame.display.update()
        time.sleep(SLEEP_DURATION)
        environment.visible_targets()

    # display the tracks which the agents have recorded

    for i in environment.agents:
        sensing_agent = environment.agents[i]
        import_agent_record(screen, sensing_agent.export_tracks())
    pygame.display.update()

    # wait for the user to click their mouse to exit
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)

    # initialize sensing agent
    sa = init_sensing_agent(origin=(500, 200), width=np.pi)
    sa.rotate_agent((500, 500))
    sa.heartbeat()
    sa.avg_window_len = 1
    sensing_agents = {}
    sensing_agents[sa._id] = sa

    targets = []
    # load json point files and initialize targets
    for i, file in enumerate(sys.argv[1:]):
        p = load_json_file(file)
        t = init_target(_id=i, path=p)
        targets.append(t)

    # initialize environment
    env = init_environment(sensing_agents=sensing_agents, targets=targets)

    tracking_test(screen, env)


if __name__ == "__main__":
    main()
