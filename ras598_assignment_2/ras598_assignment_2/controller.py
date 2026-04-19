#!/usr/bin/env python3

import math


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def compute_heading_to_target(rx, ry, tx, ty):
    return math.atan2(ty - ry, tx - rx)


def compute_turn_go_turn_cmd(
    robot_x,
    robot_y,
    robot_yaw,
    target_x,
    target_y,
    prev_state=None,
    goal_tol=0.45,
    max_v=0.55,
    max_w=1,
):
    ALIGN_TOL = 0.02
    DRIFT_LIMIT = 0.18

    dist = compute_distance(robot_x, robot_y, target_x, target_y)

    if dist < goal_tol:
        return {
            'done': True,
            'state': 'done',
            'linear': 0.0,
            'angular': 0.0,
            'dist': dist,
            'heading_err': 0.0,
        }

    target_heading = compute_heading_to_target(robot_x, robot_y, target_x, target_y)
    err = normalize_angle(target_heading - robot_yaw)

    if prev_state == 'drive' and abs(err) < DRIFT_LIMIT:
        v = min(max_v, 0.9 * dist)
        return {
            'done': False,
            'state': 'drive',
            'linear': v,
            'angular': 0.0,
            'dist': dist,
            'heading_err': err,
        }

    if abs(err) > ALIGN_TOL:
        w = max(-max_w, min(max_w, 2.6 * err))
        return {
            'done': False,
            'state': 'rotate',
            'linear': 0.0,
            'angular': w,
            'dist': dist,
            'heading_err': err,
        }

    v = min(max_v, 0.9 * dist)
    return {
        'done': False,
        'state': 'drive',
        'linear': v,
        'angular': 0.0,
        'dist': dist,
        'heading_err': err,
    }