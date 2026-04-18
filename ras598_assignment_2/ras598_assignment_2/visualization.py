#!/usr/bin/env python3

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def make_line_strip_marker(marker_id, frame_id, points_xy, rgb, width=0.05):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'planner'
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = width

    marker.color.r = float(rgb[0])
    marker.color.g = float(rgb[1])
    marker.color.b = float(rgb[2])
    marker.color.a = 1.0

    marker.lifetime = Duration(sec=0, nanosec=0)

    for x, y in points_xy:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.05
        marker.points.append(p)

    return marker


def make_sphere_marker(marker_id, frame_id, x, y, rgb, scale=0.18):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'planner'
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.08
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    marker.color.r = float(rgb[0])
    marker.color.g = float(rgb[1])
    marker.color.b = float(rgb[2])
    marker.color.a = 1.0

    marker.lifetime = Duration(sec=0, nanosec=0)

    return marker


def build_marker_array(raw_path_world, pruned_path_world=None, current_goal_world=None, frame_id='map'):
    markers = MarkerArray()

    markers.markers.append(
        make_line_strip_marker(
            marker_id=0,
            frame_id=frame_id,
            points_xy=raw_path_world,
            rgb=(0.0, 1.0, 0.0),
            width=0.05,
        )
    )

    if pruned_path_world is None:
        pruned_path_world = raw_path_world

    markers.markers.append(
        make_line_strip_marker(
            marker_id=1,
            frame_id=frame_id,
            points_xy=pruned_path_world,
            rgb=(0.0, 0.0, 1.0),
            width=0.07,
        )
    )

    if current_goal_world is None and len(pruned_path_world) > 0:
        current_goal_world = pruned_path_world[-1]

    if current_goal_world is not None:
        markers.markers.append(
            make_sphere_marker(
                marker_id=2,
                frame_id=frame_id,
                x=current_goal_world[0],
                y=current_goal_world[1],
                rgb=(1.0, 0.0, 0.0),
                scale=0.2,
            )
        )

    return markers