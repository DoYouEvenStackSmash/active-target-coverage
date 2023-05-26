#!/usr/bin/python3
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from render_support import TransformFxns as tfn
import pygame
import time
import sys
import numpy as np
import json


def create_circle(center_pt, radius=50, num_pts=50):
    step = 2 * np.pi / num_pts
    counter = num_pts // 2
    angles = []
    pts = []

    for i in range(num_pts//2):
        angles.append(i * step)
    for i in range(num_pts//2):
        angles.append(-np.pi + i * step)

    # for i, a in enumerate(angles):
    
    #     angles[i] = a + np.pi

    for i in range(len(angles)):
        pt = mfn.pol2car(center_pt, radius, angles[i])
        pts.append(pt)
    return pts


def create_line(pt1, pt2, num_pts=50):
    pts = []
    theta, r = mfn.car2pol(pt1, pt2)
    step = r / num_pts
    for i in range(1, num_pts):
        pts.append(mfn.pol2car(pt1, step * i, theta))
    return pts


def render_path(screen, paths):
    for path in paths:
        # pafn.clear_frame(screen)
        for pt in path:
            print(pt)
            pafn.frame_draw_dot(screen, pt, pafn.colors["magenta"])
            pygame.display.update()
            time.sleep(0.02)
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                f = open("out.json", "w")
                path = {"points": [{"x": p[0], "y": p[1]} for p in paths[0]]}
                f.write(json.dumps(path, indent=2))
                f.close()
                sys.exit()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    pafn.clear_frame(screen)
    paths = []
    origin = (500, 500)
    circ = create_circle(origin, 100, 50)
    # circ.reverse()
    intersection_pt = mfn.pol2car(origin, 100, 0)
    pafn.frame_draw_dot(screen, intersection_pt, pafn.colors["cyan"])
    pygame.display.update()

    print(paths)
    start_pt = mfn.pol2car(intersection_pt, 300,  -np.pi / 2)
    end_pt = mfn.pol2car(intersection_pt, 300, np.pi / 2)
    paths.append(create_line(start_pt, intersection_pt, 25))
    paths.append(circ)
    paths.append(create_line(intersection_pt, end_pt, 25))
    super_path = []
    for path in paths:
        for pt in path:
            super_path.append(pt)
    paths = [super_path]

    render_path(screen, paths)


if __name__ == "__main__":
    main()
