#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

LALT = 256
LSHIFT = 1


def stepwise_prediction_test(screen, environment):
    last_pt = None
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == LALT:
                    # while pygame.MOUSEBUTTONUP not in [
                    #     event.type for event in pygame.event.get()
                    # ]:
                    #     continue
                    # pafn.clear_frame(screen)
                    for k in environment.agents:
                        sa = environment.agents[k]
                        sa.heartbeat()
                        render_predictions(screen, sa)
                    pygame.display.update()
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    for k in environment.agents:
                        sa = environment.agents[k]
                        sa.heartbeat()

                    p = pygame.mouse.get_pos()
                    if p == last_pt:
                        continue
                    last_pt = p
                    det = Detection(
                        Position(1, p[0], p[1]), YoloBox(0, [10, 10, 0.1, 0.1], 0)
                    )
                    for t in environment.targets:
                        t.path.append(det)
                        t.step()
                    environment.visible_vertical_targets()
                else:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    if p == last_pt:
                        continue
                    last_pt = p
                    det = Detection(
                        Position(1, p[0] / 1000, p[1] / 1000),
                        YoloBox(0, [10, 10, 0.1, 0.1], 0),
                    )
                    for t in environment.targets:
                        t.path.append(det)
                        t.step()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    sa = init_sensing_agent(origin=Position(500, 500, 0))
    zero_det = Detection(Position(0, 0, 0), YoloBox(0, [10, 10, 0.1, 0.1], 0))
    t = Target(zero_det)
    sensing_agents = {sa._id: sa}
    env = init_environment(sensing_agents=sensing_agents, targets=[t])
    stepwise_prediction_test(screen, env)


if __name__ == "__main__":
    main()
