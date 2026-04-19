#!/usr/bin/env python3

import math

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from example_interfaces.srv import Trigger
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

from ras598_assignment_2.map_utils import (
    build_occupancy_grid,
    world_to_grid,
    grid_to_world,
    is_in_bounds,
    is_occupied,
)
# from ras598_assignment_2.astar import astar_search, prune_path, has_line_of_sight
from ras598_assignment_2.astar import astar_search, prune_path
from ras598_assignment_2.visualization import build_marker_array
from ras598_assignment_2.controller import compute_turn_go_turn_cmd


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw (rotation around Z) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner_markers', 10)

        self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)
        self.create_subscription(Float32, '/energy_consumed', self.energy_callback, 10)

        self.task_client = self.create_client(Trigger, '/get_task')

        pkg = get_package_share_directory('ras598_assignment_2')
        self.map_info = build_occupancy_grid(
            package_share=pkg,
            grid_resolution=0.2,
            inflation_radius_m=0.8,
        )

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.latest_energy = None

        self.raw_path_world = []
        self.pruned_path_world = []
        self.wp_idx = 0
        self.done = False
        self.ctrl_state = None

        self.get_logger().info('Planner node started, requesting task...')
        self.request_task_and_plan()

        self.create_timer(0.5, self.publish_markers)
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def energy_callback(self, msg: Float32):
        self.latest_energy = msg.data

    def request_task_and_plan(self):
        self.get_logger().info('Waiting for /get_task service...')
        self.task_client.wait_for_service()

        future = self.task_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if not resp or not resp.success:
            self.get_logger().error('Failed to get task from grading scout.')
            return

        try:
            sx, sy, gx, gy = [float(v.strip()) for v in resp.message.split(',')]
        except ValueError as e:
            self.get_logger().error(f'Could not parse task string: {e}')
            return

        self.get_logger().info(f'Task received: start=({sx:.1f},{sy:.1f}) goal=({gx:.1f},{gy:.1f})')
        self.plan_path((sx, sy), (gx, gy))

    def plan_path(self, start_world, goal_world):
        start_grid = world_to_grid(start_world[0], start_world[1], self.map_info)
        goal_grid = world_to_grid(goal_world[0], goal_world[1], self.map_info)

        self.get_logger().info(f'Grid start={start_grid}, goal={goal_grid}')

        for label, gc in [('Start', start_grid), ('Goal', goal_grid)]:
            if not is_in_bounds(gc[0], gc[1], self.map_info):
                self.get_logger().error(f'{label} cell {gc} is out of bounds.')
                return
            if is_occupied(gc[0], gc[1], self.map_info):
                self.get_logger().error(f'{label} cell {gc} is occupied after inflation!')
                return

        raw_path = astar_search(start_grid, goal_grid, self.map_info)
        if raw_path is None:
            self.get_logger().error('A* found no path.')
            return

        pruned_path = prune_path(raw_path, self.map_info)

        self.raw_path_world = [grid_to_world(c[0], c[1], self.map_info) for c in raw_path]
        self.pruned_path_world = [grid_to_world(c[0], c[1], self.map_info) for c in pruned_path]

        if self.raw_path_world:
            self.raw_path_world[0] = start_world
            self.raw_path_world[-1] = goal_world

        if self.pruned_path_world:
            self.pruned_path_world[0] = start_world
            self.pruned_path_world[-1] = goal_world

        self.wp_idx = 1 if len(self.pruned_path_world) > 1 else 0
        self.done = False
        self.ctrl_state = None

        self.get_logger().info(f'Path found: {len(raw_path)} raw nodes -> {len(pruned_path)} after pruning')
        for i, (wx, wy) in enumerate(self.pruned_path_world):
            self.get_logger().info(f'  wp[{i}] = ({wx:.2f}, {wy:.2f})')


    def publish_markers(self):
        if not self.raw_path_world:
            return

        if self.pruned_path_world and self.wp_idx < len(self.pruned_path_world):
            current_goal = self.pruned_path_world[self.wp_idx]
        elif self.pruned_path_world:
            current_goal = self.pruned_path_world[-1]
        else:
            current_goal = None

        markers = build_marker_array(
            raw_path_world=self.raw_path_world,
            pruned_path_world=self.pruned_path_world,
            current_goal_world=current_goal,
            frame_id='map',
        )
        self.marker_pub.publish(markers)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.robot_x is None:
            return

        if not self.pruned_path_world or self.done:
            self.stop_robot()
            return

        if self.wp_idx >= len(self.pruned_path_world):
            self.done = True
            self.get_logger().info('All waypoints done.')
            if self.latest_energy is not None:
                self.get_logger().info(f'Final energy: {self.latest_energy:.3f} units')
            self.stop_robot()
            return

        tx, ty = self.pruned_path_world[self.wp_idx]
        is_last_wp = (self.wp_idx == len(self.pruned_path_world) - 1)

        result = compute_turn_go_turn_cmd(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            target_x=tx,
            target_y=ty,
            prev_state=self.ctrl_state,
            goal_tol=0.50 if is_last_wp else 0.40,
            max_v=0.55,
            max_w=0.75,
        )

        self.ctrl_state = result['state']

        if result['done']:
            self.get_logger().info(f'Reached wp[{self.wp_idx}] at ({tx:.2f}, {ty:.2f})')
            self.wp_idx += 1

            if self.wp_idx >= len(self.pruned_path_world):
                self.done = True
                self.get_logger().info('All waypoints done.')
                if self.latest_energy is not None:
                    self.get_logger().info(f'Final energy: {self.latest_energy:.3f} units')
                self.stop_robot()
                return

            tx, ty = self.pruned_path_world[self.wp_idx]
            next_is_last = (self.wp_idx == len(self.pruned_path_world) - 1)

            result = compute_turn_go_turn_cmd(
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                robot_yaw=self.robot_yaw,
                target_x=tx,
                target_y=ty,
                prev_state='drive',
                goal_tol=0.50 if next_is_last else 0.40,
                max_v=0.55,
                max_w=0.75,
            )

            self.ctrl_state = result['state']
            cmd = Twist()
            cmd.linear.x = result['linear']
            cmd.angular.z = result['angular']
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()
        cmd.linear.x = result['linear']
        cmd.angular.z = result['angular']
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()