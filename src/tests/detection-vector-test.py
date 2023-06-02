#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from env_init import *

from drawing_functions import *
def create_detections_without_range(
                                time_of_detection,
                                detection_cls,
                                x,
                                y,
                                w,
                                h,
                                img_shape_x,
                                img_shape_y,
                                sensor_fov_width,
                                sensor_fov_height):
    """
    wrapper for Yolo style detections
    """
    # calculate vector 1, agent pov on horizontal plane
    """
                  (0,0)
            +-------+-------+
            |       |       |
            |       |       |
            |       |       |
    (0,0)   |__ B<--A_______| (100,0)
            |   |   |       |
            |   v   |       |
            |   C   |       |
            +-------+-------+
                  (0,100)
    Agent frame of reference
    image_shape: 1000
    sensor_fov_width: pi / 2
    rel_x = 25
    theta = -np.pi / 4
    """
    dist = 0
    # normalize x between 0 and 100
    rel_x = (x * img_shape_x - (img_shape_x / 2)) / img_shape_x * 100 + 50
    # normalize theta in terms of agent pov
    theta = (rel_x / 100) * sensor_fov_width - (sensor_fov_width / 2)
    
    # vertical component
    rel_y = (y * img_shape_y - (img_shape_y / 2)) / img_shape_y * 100 + 50
    phi = (rel_y / 100) * sensor_fov_height - (sensor_fov_height / 2)

    # bbox normalized
    bbox = [rel_x, rel_y, w, h]

    posn = Position(dist, rel_x, rel_y, theta, phi)
    yb = sann.register_annotation(detection_cls, [rel_x, rel_y, w, h], time_of_detection)
    det = Detection(posn, yb)
    return det
  
def map_detection_back(angle, max_angle, max_coord = 1000):
  ratio = angle / max_angle
  val = ratio * max_coord + max_coord/2
  return val

def detection_vector_test(screen):
  last_pt = None
  while 1:
    for event in pygame.event.get():
      pt = pygame.mouse.get_pos()
      if pt == last_pt:
        continue
      last_pt = pt
      pafn.clear_frame(screen)
      

      dim = 0.15
      max_x = 1000
      det = create_detections_without_range(1, 1, pt[0]/max_x, pt[1]/max_x,0.15, 0.15, 1000,1000, np.pi, np.pi)
      r,az,el = det.get_cartesian_coordinates()
      theta, phi = det.get_angles()
      x = map_detection_back(theta, np.pi)
      y = map_detection_back(phi, np.pi)
      angle, dist = mfn.car2pol((500,500,0), (x,y,0))
      line_pt = mfn.pol2car((500,500,0), dist, angle)
      print(theta, phi)
      # print(az)
      
      npt = [x,y,0]
      # print(npt)
      pafn.frame_draw_dot(screen, gfn.reduce_dimension(npt), pafn.colors["green"])
      pafn.frame_draw_bold_line(screen, ((500,500),(x, 500)), pafn.colors["cyan"])
      pafn.frame_draw_cross(screen, (x,500,0), pafn.colors["indigo"])
      pafn.frame_draw_bold_line(screen, ((x, 500),(x, y)), pafn.colors["magenta"])
      pafn.frame_draw_bold_line(screen, ((500,500), line_pt), pafn.colors["indigo"])
      pygame.display.update()

    


def main():
  pygame.init()
  screen = pafn.create_display(1000,1000)
  
  pygame.display.update()
  detection_vector_test(screen)

if __name__ == '__main__':
  main()