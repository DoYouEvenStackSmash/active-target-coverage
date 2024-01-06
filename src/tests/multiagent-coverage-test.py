#!/usr/bin/python3
"""
Legacy test which allows the agent to see the target at every time step
"""
import sys

sys.path.append("../")
sys.path.append("../loco_format/")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

# from loco_format.LOCO import *
from bbox_extractor import *

SLEEP_DURATION = 0.05


def wave_test(screen, environment, imwaves, measurement_rate=1):
    """
    A test for tracking the user's mouse according to some measurement rate
    """
    pt = None
    last_pt = None
    counter = 0
    curr_pts = []
    pred_pts = []
    marked_pts = []
    for i in range(1, len(imwaves), measurement_rate):
        # if i > measurement_rate:
        
        for j in range(measurement_rate):
            pafn.clear_frame(screen)
            environment_agent_update(environment, True)
            
            # if i > measurement_rate * 2:
            environment_agent_illustration(
                screen, environment, measurement_rate, curr_pts, pred_pts, marked_pts
            )
            environment_target_illustration(screen, environment)
        pygame.display.update()
            # i += 1
            # environment_agent_update(environment, True)
        # if not i % measurement_rate:
        target_arr = []
        for w in range(len(imwaves[i][1])):
            target = imwaves[i][1][w]
            target_arr.append(
                init_target(_id=i * w + w, origin=(target[0], target[1]))
            )
            target_arr[-1].attributes = target
        environment.targets = target_arr
        environment.target_coverage(imwaves[i][0], measurement_rate)
        # environment_agent_update(environment, True)
        
        time.sleep(SLEEP_DURATION)

    # serialize the agent tracks for viewing
    # for i in environment.agents:
    #     sensing_agent = environment.agents[i]
    #     import_agent_record(screen, sensing_agent.export_tracks())
    # pygame.display.update()

    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                environment.serialize_agent_tracks()
                return
    sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1920, 1080)
    pafn.clear_frame(screen)

    # initialize sensing agent
    sa = init_sensing_agent(origin=(860, -100), width=2 * np.pi, radius=7000)
    sa.rotate_agent((860, 500))

    sa.heartbeat()
    sa.obj_tracker.avg_window_len = 4
    sa.obj_tracker.track_lifespan = 10
    sa.obj_tracker.radial_exclusion = 200
    sa.ALLOW_ROTATION = False
    sa.ALLOW_TRANSLATION = False
    sensing_agents = {}
    sensing_agents[sa._id] = sa
    imdict = load_waves(sys.argv[1])
    # waves = load_waves(sys.argv[1])
    imwaves = []
    for k, v in imdict.items():
        imwaves.append([k, v])

    # initialize environment
    env = init_environment()#sensing_agents=sensing_agents)
    target_arr = []
    for w in range(len(imwaves[0][1])):
        target = imwaves[0][1][w]
        target_arr.append(init_target(_id=w, origin=(target[0], target[1])))
        target_arr[-1].attributes = target
    env.targets = target_arr
    env.target_coverage(imwaves[0][0], int(sys.argv[-1]))
    # environment_agent_update(env, True)
    # environment_agent_update(env, True)
    wave_test(screen, env, imwaves, int(sys.argv[-1]))


if __name__ == "__main__":
    main()
