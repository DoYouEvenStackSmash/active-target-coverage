#!/usr/bin/python3

import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *

from support.file_loader import *

def target_traverse(screen, environment):
  """
  Step through all targets in the environment
  """
  flag = True
  while flag:
    flag = False
    for i in range(len(environment.targets)):
      t = environment.targets[i]
      pafn.frame_draw_dot(screen, t.get_position(), t.color)
      flag = t.step() or flag
    pygame.display.update()
    time.sleep(0.005)
  sys.exit()
  

def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    targets = []
    # load json point files
    for i,file in enumerate(sys.argv[1:]):
      p = load_json_file(file)
      t = init_target(_id=i, path=p)
      targets.append(t)
    env = init_environment(targets=targets)
    target_traverse(screen, env)

if __name__ == '__main__':
  main()
