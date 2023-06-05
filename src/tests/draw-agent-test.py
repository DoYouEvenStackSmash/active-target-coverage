#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *


def main():
    o = 1000
    pygame.init()
    screen = pafn.create_display(o, o)
    pafn.clear_frame(screen)
    sa = init_sensing_agent(0, origin=Position(o / 2, o / 2, 0))
    print(sa.get_origin().get_cartesian_coordinates())
    draw_sensing_agent(screen, sa)
    pygame.display.update()

    time.sleep(5)


if __name__ == "__main__":
    main()
