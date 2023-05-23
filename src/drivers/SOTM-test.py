#!/usr/bin/python3
import numpy as np
from render_support import PygameArtFxns as pafn
from render_support import GeometryFxns as gfn
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
from support.transform_polygon import *
from support.Polygon import *
from support.Link import Link

import collections

# from aux_functions import *
# from Dataloader import Dataloader
from YoloBox import YoloBox
from StreamingObjectTrackManager import ObjectTrackManager
from ObjectTrack import ObjectTrack
from AnnotationLoader import AnnotationLoader as al
from OTFTrackerApi import StreamingAnnotations as sann
from RigidBody import RigidBody
from SensingAgent import SensingAgent
from Sensor import Sensor
from Target import Target
from Environment import Environment
import sys
import json
import pygame
import time

COLLISION_THRESHOLD = 10
VERBOSE = True
SAMPLE_RATE = 400
LALT = 256
LSHIFT = 1
LCTRL = 64
SPACE = 32
OFFT = 20
SPLINE_COUNT = 2
TRANSLATE = False


def repeatable_otm(screen, otm):
    counter = 0
    origin = (500, 500)
    displacement = False
    # draw_sensing_agent(screen, sensing_agent)
    pygame.display.update()
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_mods() == SPACE:
                    continue
                elif pygame.key.get_mods() == LSHIFT:  # rotate relative
                    continue
                elif pygame.key.get_mods() == LALT:  # estimate
                    estimates = otm.get_predictions()
                    if not len(estimates):
                        continue
                    for p in estimates:
                        pafn.frame_draw_dot(
                            screen, (p[0] + 500, p[1] + 500), pafn.colors["yellow"]
                        )
                    pygame.display.update()
                    continue
                elif pygame.key.get_mods() == LCTRL:
                    center = (0, 0)
                    theta, r = mfn.car2pol(origin, center)
                    otm.add_linear_displacement(r, theta)
                    continue
                else:
                    while pygame.MOUSEBUTTONUP not in [
                        event.type for event in pygame.event.get()
                    ]:
                        continue
                    p = pygame.mouse.get_pos()
                    print(p)
                    counter += 1
                    bbox = []
                    # if displacement == True:
                    bbox = [p[0], p[1], 1, 1]

                    pafn.frame_draw_dot(screen, p, pafn.colors["green"])
                    yb_arr = [sann.register_annotation(0, bbox, f"frame_{counter}")]
                    otm.add_new_layer(yb_arr)
                    otm.process_layer(len(otm.layers) - 1)
                    pygame.display.update()


def main():
    pygame.init()
    screen = pafn.create_display(1000, 1000)
    otm = ObjectTrackManager()

    repeatable_otm(screen, otm)


if __name__ == "__main__":
    main()
