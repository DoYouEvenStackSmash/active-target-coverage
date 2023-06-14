#!/usr/bin/python3
import sys

sys.path.append("../")
sys.path.append(".")
from render_support import MathFxns as mfn
from render_support import GeometryFxns as gfn
from render_support import PygameArtFxns as pafn
from render_support import TransformFxns as tfn
import pygame
import time

import numpy as np
import json

MAGNITUDE = 6


def create_circle(center_pt, radius=50, num_pts=50, direction=1):
    """
    Creates a circle of points in euclidean coordinates around the center
    return a list of (x,y) points
    """

    step = 2 * np.pi / num_pts
    counter = num_pts // 2
    angles = []
    pts = []

    angles = []
    angle1 = []
    angle2 = []
    for i in range(num_pts // 2):
        angle1.append(i * step)
    for i in range(num_pts // 2):
        angle2.append(-np.pi + i * step)
    if direction < 0:
        temp = angle1
        angle1 = angle2
        angle2 = temp
    for i in angle1:
        angles.append(i)
    for i in angle2:
        angles.append(i)
    for i in range(len(angles)):
        pt = mfn.pol2car(center_pt, radius, angles[i])
        pts.append(pt)
    return pts


def create_line(pt1, pt2, num_pts=50):
    """
    Creates a straight line of num_points between pt1 and pt2
    returns a list of (x,y) points
    """
    pts = []
    theta, r = mfn.car2pol(pt1, pt2)
    step = r / num_pts
    for i in range(1, num_pts):
        pts.append(mfn.pol2car(pt1, step * i, theta))
    return pts


def render_path(screen, paths, filename="out.json", magnitude=MAGNITUDE):
    """
    Renders and serializes a path
    """

    MAGNITUDE = int(magnitude)

    rng = np.random.default_rng(12345)

    rand_angle = lambda: (rng.uniform() * 2 * np.pi) - np.pi

    rand_mag = lambda: (rng.uniform() * MAGNITUDE)

    for p in range(len(paths)):
        path = paths[p]
        for i, pt in enumerate(path):
            print(pt)
            path[i] = mfn.pol2car(pt, rand_mag(), rand_angle())
            pafn.frame_draw_dot(screen, path[i], pafn.colors["magenta"])
        pygame.display.update()
        # time.sleep(0.02)

    f = open(filename, "w")
    path = {"points": [{"x": p[0], "y": p[1]} for p in paths[0]]}
    f.write(json.dumps(path, indent=2))
    f.close()
    sys.exit()


def create_grid(origin=(0, 0), width=1000, height=1000, points=10):
    """
    Creates a grid of points
    returns a list of (x,y) points
    """
    height_endpoint = mfn.pol2car(origin, height, np.pi / 2)
    vertical_axis = create_line(origin, height_endpoint, points)
    lines = []
    for pt in vertical_axis:
        horizontal_endpoint = mfn.pol2car(pt, width, 0)
        horizontal_axis = create_line(pt, horizontal_endpoint, points)
        lines.append(horizontal_axis)
    pts = []
    for line in lines:
        for pt in line:
            pts.append(pt)
    return [pts]


def loop_line():
    """
    Creates a line with a loop in the center
    Returns a list of (x,y) points
    """
    paths = []
    origin = (500, 500)
    circ = create_circle(origin, 100, 400)
    # circ.reverse()
    circ2 = create_circle((700, 500), 100, 400, -1)
    circ2.reverse()
    intersection_pt = mfn.pol2car(origin, 100, 0)

    print(paths)
    start_pt = mfn.pol2car(intersection_pt, 300, -np.pi / 2)
    end_pt = mfn.pol2car(intersection_pt, 600, np.pi / 2)
    paths.append(create_line(start_pt, intersection_pt, 400))
    paths.append(circ)
    paths.append(circ2)
    paths.append(create_line(intersection_pt, end_pt, 400))
    super_path = []
    for path in paths:
        for pt in path:
            super_path.append(pt)
    paths = [super_path]
    return paths


def get_lerp(origin, p):
    segment = 4
    pts = []
    if segment > 0:
        spts = []
        pts = []
        ev = []
        l1_pts = []
        l2_pts = []

        seg_step = 1 / segment
        # break origin-target segment into regions
        for i in range(segment):
            spts.append(gfn.lerp(origin, p, i * seg_step))
        spts.append(p)

        # get equilateral vertex of each segment
        sign = 1
        for i in range(len(spts) - 1):
            ev.append(gfn.get_isosceles_vertex(spts[i], spts[i + 1], sign, 35))
            sign = sign * -1

        # calculate lerp points
        n = 400
        step = 1 / n
        print(len(spts))
        for sp in range(len(spts) - 1):
            for i in range(n):
                # lerp between origin and equilateral vertex
                l1 = gfn.lerp(spts[sp], ev[sp], step * i)
                # lerp between equilateral vertex and target
                l2 = gfn.lerp(ev[sp], spts[sp + 1], step * i)
                # lerp between lerps
                m1 = gfn.lerp(l1, l2, step * i)

                l1_pts.append(l1)
                l2_pts.append(l2)
                pts.append(m1)
    return pts


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1400)
    pafn.clear_frame(screen)
    paths = None
    flags = sys.argv[1:]
    # if len(sys.argv) == 2:
    #     sys.argv.append("0")
    opts = ["ACC", "GRID", "CIRCLE"]
    for f in flags:
        if f == "ACC":
            paths = loop_line()
            render_path(screen, paths, f"loop_{sys.argv[-1]}noise.json", sys.argv[-1])
        elif f == "GRID":
            paths = create_grid()
            render_path(screen, paths, f"grid_{sys.argv[-1]}noise.json", sys.argv[-1])
        elif f == "CIRCLE":
            origin = (500, 500)
            paths = create_circle(origin, 150, 50)
            render_path(
                screen, [paths], f"circle_{sys.argv[-1]}noise.json", sys.argv[-1]
            )
        elif f == "LERP":
            origin = (800, 200)
            dest = (400, 1000)
            paths = get_lerp(origin, dest)
            render_path(screen, [paths], f"lerp_{sys.argv[-1]}noise.json", sys.argv[-1])
        elif f == "LINE":
            origin = (650, 200)
            dest = (400, 1000)
            paths = create_line(origin, dest)
            render_path(screen, [paths], f"line_{sys.argv[-1]}noise.json", sys.argv[-1])
        else:
            print(f"options: {opts}")
            sys.exit()


if __name__ == "__main__":
    main()
