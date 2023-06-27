#!/usr/bin/python3
import json
import sys

sys.path.append(".")
sys.path.append("../")
from render_support import MathFxns as mfn
from render_support import TransformFxns as tfn
import flatbuffers

# import matplotlib.pyplot as plt
from LOCO.TopLoco import *


def adjust_angle(theta):
    """adjusts some theta to arctan2 interval [0,pi] and [-pi, 0]"""
    if theta > np.pi:
        theta = theta + -2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    return theta


def undo_angle_displacement(agent_coord_pt, prev_state, curr_state):
    pt = agent_coord_pt
    prev_theta = prev_state.orientation
    curr_theta = curr_state.orientation
    if prev_theta == curr_theta:
        return pt
    # if prev_theta / curr_theta < 0:
    prev_theta = prev_theta if prev_theta >= 0 else prev_theta + 2 * np.pi
    curr_theta = curr_theta if curr_theta >= 0 else curr_theta + 2 * np.pi
    # rotate by the difference between old and new
    angle = adjust_angle(prev_theta - curr_theta)
    rot_mat = tfn.calculate_rotation_matrix(angle, 1)
    new_pt = tfn.rotate_point(
        (prev_state.position.x, prev_state.position.y), pt, rot_mat
    )
    return new_pt


def undo_point_translation(pt, fov_width, prev_state, curr_state):
    # displace by the same magnitude in the opposite direction
    prev_posn = prev_state.position.x, prev_state.position.y
    curr_posn = curr_state.position.x, curr_state.position.y

    global_angle = lambda x, fov_width, orientation: adjust_angle(
        x / 100 * fov_width - fov_width / 2 + orientation
    )
    global_posn = lambda origin, r, agent_angle: mfn.pol2car(origin, r, agent_angle)
    origin = curr_posn
    # theta, rad = mfn.car2pol(prev_posn,curr_posn)
    # return (pt[0],pt[1] + rad)
    agent_angle = global_angle(pt[0], fov_width, curr_state.orientation)
    curr_rel_posn = global_posn(origin, pt[1], agent_angle)
    if prev_posn == curr_posn:
        print("no translation")
        return curr_rel_posn

    theta, rad = mfn.car2pol(prev_posn, curr_posn)
    # # mfn.pol2car()
    new_pt = mfn.pol2car(curr_rel_posn, -rad, adjust_angle(theta + np.pi))
    # theta, rad = mfn.car2pol(new_pt, prev_posn)
    return new_pt
    # return new_pt


def convert_back_to_world_coordinates(filename):
    buf = open(filename, "rb").read()
    buf = bytearray(buf)
    # get toploco
    b = TopLoco.GetRootAsTopLoco(buf, 0)
    top_loco_t = TopLocoT.InitFromObj(b)
    # get linked tracks
    linked_tracks = top_loco_t.linkedTracks
    annos = top_loco_t.annotations
    fov_width = top_loco_t.sensorParams.fovWidth
    global_angle = lambda x, fov_width, orientation: adjust_angle(
        x / 100 * fov_width - fov_width / 2 + orientation
    )
    global_posn = lambda origin, r, agent_angle: mfn.pol2car(origin, r, agent_angle)
    states = top_loco_t.states
    trkmap = {}
    for anno in annos:
        trackmapIndex = anno.trackmapIndex
        if trackmapIndex not in trkmap:
            trkmap[trackmapIndex] = []
        # print()
        # print(anno)

        state = states[anno.stateId - 1]

        x, y, w, h = anno.bbox.x, anno.bbox.y, anno.bbox.w, anno.bbox.h
        posn = anno.position
        # x is distance from origin point, y is sideways distance from center axis
        rect_x, rect_y = posn.x, posn.y
        orientation = state.orientation
        theta, rad = mfn.car2pol((0, 0), (rect_x, rect_y))
        agent_angle = adjust_angle(orientation + theta)
        # agent_angle = global_angle(x, fov_width, orientation)
        origin = (state.position.x, state.position.y)
        pt = global_posn(origin, y, agent_angle)
        target_posn = pt

        # state = states[anno.stateId - 2]

        # origin = (state.position.x,state.position.y)

        # target_posn = global_posn(origin, y, agent_angle)
        trkmap[trackmapIndex].append({"x": target_posn[0], "y": target_posn[1]})
    for t in trkmap:
        f = open(f"track_{str(t)}.json", "w")
        f.write(json.dumps({"points": trkmap[t]}, indent=2))
        f.close()


if __name__ == "__main__":
    convert_back_to_world_coordinates(sys.argv[-1])
