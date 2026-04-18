# autonomous-cave-navigation-astar

ROS 2 motion planning stack for autonomous navigation in a cave-like Stage environment using inflated occupancy grids, A* path planning, line-of-sight path pruning, and a Turn-Go-Turn local controller with RViz visualization and energy-aware execution.

## Overview

This project implements a complete 2D navigation pipeline for a robot operating in a cave environment in the Stage simulator. The system loads a bitmap map, converts it into an inflated occupancy grid, plans a collision-free path with A*, prunes unnecessary waypoints using line-of-sight checks, and executes the path with a simple state-machine controller.

The project was built in ROS 2 Jazzy and follows the structure and interfaces required for a motion planning assignment based on Stage simulation, RViz visualization, and an external grading scout.

## What this repository contains

```text
autonomous-cave-navigation-astar/
├── ras598_assignment_2/
│   ├── cave_filled.png
│   ├── grading_scout.py
│   ├── map.yaml
│   ├── package.xml
│   ├── planner_launch.py
│   ├── planning.rviz
│   ├── README.md
│   ├── resource/
│   │   └── ras598_assignment_2
│   ├── setup.cfg
│   ├── setup.py
│   └── ras598_assignment_2/
│       ├── __init__.py
│       ├── astar.py
│       ├── controller.py
│       ├── map_utils.py
│       ├── planner_node.py
│       └── visualization.py
├── LICENSE
└── README.md
````

## Main components

### `map_utils.py`

Loads the map, parses `map.yaml`, inflates obstacles, builds the 0.2 m occupancy grid, and provides coordinate transforms between world coordinates and grid coordinates.

### `astar.py`

Implements A* from scratch on an 8-connected grid. It also includes Bresenham-based line-of-sight checks and path pruning.

### `controller.py`

Implements the local Turn-Go-Turn controller. The robot rotates until aligned with the next waypoint, then drives with zero angular velocity.

### `planner_node.py`

Main ROS 2 node. It:

* requests the task from `/get_task`
* builds the occupancy grid
* plans the global path
* prunes the path
* publishes `/cmd_vel`
* subscribes to `/ground_truth` and `/energy_consumed`
* publishes RViz markers on `/planner_markers`

### `visualization.py`

Builds the required RViz markers:

* green `LINE_STRIP` for the raw A* path
* blue `LINE_STRIP` for the pruned path
* red `SPHERE` for the current local goal

### `planner_launch.py`

Launches the complete navigation stack:

* Stage simulator
* `map_server`
* lifecycle manager for `map_server`
* planner node
* grading scout
* RViz

## ROS version and workspace assumptions

This project was developed with:

* Ubuntu 24.04
* ROS 2 Jazzy
* workspace layout using `~/ros2_ws/src`
* `colcon build --symlink-install`

Environment setup used during development:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## How to clone and run from scratch

### 1. Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone dependencies

```bash
git clone https://github.com/ras-mobile-robotics/Stage.git
git clone https://github.com/ras-mobile-robotics/stage_ros2.git
git clone https://github.com/Tatwik19/autonomous-cave-navigation-astar.git
```

### 3. Move into the workspace root

```bash
cd ~/ros2_ws
```

### 4. Install system and ROS dependencies

```bash
sudo apt update
sudo apt install libfltk1.3-dev ros-jazzy-ackermann-msgs ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager -y
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 6. Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 7. Launch the complete stack

```bash
ros2 launch ras598_assignment_2 planner_launch.py
```

This launch file starts:

* Stage with the cave world
* `map_server`
* lifecycle manager
* planner node
* grading scout
* RViz

## Package naming and ROS interfaces

The package name is:

```text
ras598_assignment_2
```

The planner uses the following ROS interfaces.

### Subscribed topics

* `/ground_truth` (`nav_msgs/Odometry`)
* `/energy_consumed` (`std_msgs/Float32`)

### Published topics

* `/cmd_vel` (`geometry_msgs/Twist`)
* `/planner_markers` (`visualization_msgs/MarkerArray`)

### Service client

* `/get_task`

These names were kept fixed so the planner remains compatible with the grading scout.

## Planning pipeline

## 1. Map loading and obstacle inflation

The cave environment is defined by:

* `cave_filled.png`
* `map.yaml`

The map image is loaded in grayscale. Dark pixels are treated as obstacles. Obstacles are then inflated to account for robot footprint and safety margin. After inflation, the map is converted into a coarse occupancy grid where each cell represents:

```text
0.2 m × 0.2 m
```

This grid is what A* searches over.

## 2. Global planning with A*

A* is implemented from scratch on an 8-connected grid.

Neighbor motion model:

* 4 cardinal moves with cost `1.0`
* 4 diagonal moves with cost `sqrt(2)`

Heuristic:

* Euclidean distance to the goal

The planner also blocks diagonal corner cutting. This prevents the robot from slipping unrealistically between two occupied neighbors at a corner.

## 3. Path pruning

The raw A* path can contain many short grid-to-grid steps. Driving to every one of them would increase stop-and-go behavior.

To reduce this, the planner prunes the raw path using line-of-sight checks:

* if a farther waypoint is directly visible from the current anchor cell, intermediate nodes are skipped
* the pruned path usually has many fewer vertices than the raw path

This improves path smoothness and reduces unnecessary heading resets.

## 4. Local control with Turn-Go-Turn

The controller operates as a simple state machine with two main states:

### Rotate

If the heading error to the current waypoint is above a threshold, the robot rotates in place.

### Drive

Once the heading error is small enough, the robot drives straight toward the waypoint with angular velocity set to exactly `0.0`.

This design helps reduce angular jitter and keeps the local behavior easy to explain and debug.

## Coordinate math

One of the most important parts of the project is mapping between image coordinates, world coordinates, and grid coordinates.

### Image coordinates

For the image:

* origin is at the top-left corner
* `x` increases to the right
* `y` increases downward

### World coordinates

For the simulator:

* origin is defined by `map.yaml`
* in this map, the world origin is centered using the map origin values
* `x` increases to the right
* `y` increases upward

### Grid coordinates

For planning:

* the occupancy grid origin is treated at the bottom-left of the world map
* each grid cell has size `grid_resolution`

### World to grid

If `(wx, wy)` is a point in the world frame, then the corresponding grid cell is:

```text
gx = floor((wx - origin_x) / grid_resolution)
gy = floor((wy - origin_y) / grid_resolution)
```

This is used to place the start and goal on the planning grid and also to convert robot pose into planning coordinates during waypoint skipping.

### Grid to world

If `(gx, gy)` is a grid cell, the corresponding cell-center world coordinate is:

```text
wx = origin_x + (gx + 0.5) * grid_resolution
wy = origin_y + (gy + 0.5) * grid_resolution
```

This is used to visualize the path in RViz and to generate local planning waypoints.

### Why the image Y axis must be flipped

The map image stores rows from top to bottom, but the world frame increases upward. That means the image row index and world Y direction are opposite. When converting between image pixels and world/grid cells, the Y axis must be flipped to keep the planner aligned with the simulator and RViz.

## Why the robot consumed a specific amount of energy

The grading scout accumulates energy over time based on robot motion and startup events.

The total cost per tick is conceptually:

```text
cost_tick = base_drain + (|v| * linear_coeff) + (|w| * angular_coeff) + startup_tax
```

Where:

* `base_drain` is paid every tick while the system is running
* `|v|` contributes linear motion cost
* `|w|` contributes angular motion cost
* `startup_tax` is paid when the robot accelerates from standstill

This means energy increases for three main reasons:

### 1. Time spent moving

A longer route or a slower execution time increases accumulated base cost.

### 2. Turning

Angular motion is expensive. If the robot rotates frequently, overshoots, or jitters near a waypoint, energy rises quickly.

### 3. Stop-start behavior

Every time the robot comes to a full stop and starts again, it may trigger another startup penalty. This is why pruning and stable local control matter so much.

### How this planner reduces energy

This planner reduces energy by:

* pruning the raw A* path into longer straight segments
* skipping visible future waypoints when possible
* using a discrete rotate-then-drive controller
* setting angular velocity to zero while driving
* capping linear and angular speeds to avoid overshoot in narrow corridors

## Tunable parameters

The following values can be tuned to change behavior.

### In `map_utils.py`

* `grid_resolution`

  * current assignment value: `0.2`
  * smaller values give finer planning but increase search cost
* `inflation_radius_m`

  * must remain greater than `0.6`
  * larger values improve safety but may block narrow corridors

### In `controller.py`

* `ALIGN_TOL`

  * smaller value forces tighter alignment before driving
  * too small can cause extra turning
* `DRIFT_LIMIT`

  * determines how much heading drift is tolerated while staying in drive mode
  * too small causes unnecessary state switching
  * too large can cause path sloppiness
* `max_v`

  * maximum linear speed
  * higher values can overshoot in tight sections
* `max_w`

  * maximum angular speed
  * higher values turn faster but can cause oscillation

### In `planner_node.py`

* waypoint tolerance

  * tighter tolerance follows the path more precisely
  * looser tolerance can reduce stop events but may cut corners too aggressively
* dynamic waypoint skipping behavior

  * more aggressive skipping usually reduces startup penalties
  * but must still remain collision-safe through line-of-sight checks

## Typical behavior tradeoffs

### If the robot bumps walls or cuts corners

* increase `inflation_radius_m`
* reduce `max_v`
* reduce waypoint tolerance slightly

### If the robot rotates too much

* loosen `DRIFT_LIMIT` slightly
* avoid overly small `ALIGN_TOL`

### If the robot is safe but slow

* increase `max_v` carefully
* keep pruning active
* avoid unnecessary intermediate waypoints

### If the energy score is high

* check how often the robot stops
* inspect whether the pruned path has significantly fewer nodes than the raw path
* reduce oscillatory turning and overshoot near waypoints

## Visualization

The planner publishes `visualization_msgs/MarkerArray` on `/planner_markers` and displays:

* green line strip for the raw A* path
* blue line strip for the pruned path
* red sphere for the current local planner goal

This makes it easy to compare the unpruned and pruned plans during debugging and presentation.

## Example execution summary

A typical run looks like this:

* the planner requests the task from `/get_task`
* start and goal are converted from world coordinates to grid cells
* A* generates a raw path
* the raw path is pruned using line-of-sight checks
* the robot follows the pruned path with Turn-Go-Turn control
* the grading scout reports final energy after goal completion

## How this satisfies the assignment requirements

This project satisfies the main assignment requirements by:

* using a 0.2 m occupancy grid
* inflating obstacles before planning
* implementing A* from scratch
* publishing `geometry_msgs/Twist` on `/cmd_vel`
* using `/ground_truth` for exact pose feedback
* requesting the task from `/get_task`
* monitoring `/energy_consumed`
* publishing the required RViz markers on `/planner_markers`
* launching the full stack through `planner_launch.py`

## Notes

* The workspace is intended to be built with `colcon build --symlink-install`.
* If file paths differ on your machine, check `map.yaml` and `planner_launch.py`.
* RViz on some VMs may show OpenGL shader warnings. If the map and markers still render, the planning stack itself is usually unaffected.
