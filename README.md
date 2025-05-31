# 🍕 3806ICT Assignment 2 - Autonomous Pizza Delivery
A simulation project using Gazebo and Robot Operating System (ROS) where four autonomous delivery robots traverse a 16 by 16 grid world to deliver pizzas to randomly assigned delivery addresses. The system includes real-time path planning, obstacle avoidance, and communication between services via a shared world.

## 🚀 Project Overview
This project simulates a multi-robot pizza delivery system using ROS and the Gazebo simulator. Four robots deliver pizzas from a central base to customers on a 16 by 16 grid handling pathfinding, obstacle detection, and grid updates.\
Key features include:
- Pathfinding implemented using the A* algorithm.
- Real-time communication and world updates between robots and related nodes.
- Smooth interpolation between grid positions.
- Obstacle awareness via the use of sensors.

## ✏️ Authors
- Giang Pham - S5334670
- Stefan Barosan - S5277574
- Tennille Kelly - S1172755
- Will Wallace - S5287361

## 🤖 How It Works

When the system is initialised, the `dispatcher` node assigns delivery tasks to available robots, ensuring efficiency. Each task given to a robot includes the delivery address in co-ordinates and where the originating base is located. The robots receive these tasks via the `relay_server`, which acts as a bus between different components without coupling.\
Upon receiving a task, the robot will use the A* algorithm to find the most efficient path from its current position to the new location. This path is based on the latest interpretation of the world, which is maintained by the `grid_manager`. The grid manager acts as a module holding a shared memory of the robot interpretation of the world, including known obstacles and occupied cells. Each robot publishes updates to this grid continuously, especially when new obstacles are detected, which ensures that all robots have access to the most recent world interpretation.\
Movement within the environment is completed by a mix of teleportation and interpolation. Logically, the robots will move from one point to another, but visibly they will linearly interpolate between positions, emulating continuous motion within Gazebo.\
If a robot encounters a new obstacle, it reports this to the grid manager which updates the shared map. The affected robot will then recalculate its path based on this new information, ensuring adaptive navigation in altering environments.

## 🛠️ Dependencies
- ROS Noetic on an Ubuntu 20.04 LTS system.
- Gazebo 11
- Python 3
- rospy, std_msgs, geometry_msgs, nav_msgs
- VRPy
- Gazebo Plugins
- `mesa-utils` and `libgl1-mesa-dev` packages.

To install VRPy:\
` pip install vrpy `

To install the Gazebo plugins:\
` sudo apt install ros-noetic-gazebo-plugins `

To install the `mesa-utils` and `libgl1-mesa-dev` packages:\
` sudo apt install mesa-utils libgl1-mesa-dev `

## 🔧 Instructions

To run this project on your machine:
1. Clone this repository into your ROS workspace.
2. Run `catkin_make` to compile the package and `source devel/setup.bash`.
3. Launch the simulation using `roslaunch my_sim_pkg world_simulation.launch`.
4. Spawn the robots using `roslaunch my_sim_pkg spawn_robots.launch`.
5. Launch the robot control system using `roslaunch my_sim_pkg robot_system.launch`.

## 📁 Project Structure

~~~
3806ICT-Assignment-2/            ← Workspace root
└── src/
    └── my_sim_pkg/              ← Our package
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/              ← Launch files to load the world
        │   ├── robot_system.launch
        │   ├── world_simulation.launch
        │   └── spawn_robots.launch
        ├── worlds/              ← Our worlds
        │   ├── tubgbot_depot.sdf
        │   └── thumbnails/
        ├── media/               ← Our media
        │   ├── audio/
        │   ├── dem/
        │   ├── fonts/
        │   ├── gui/
        │   ├── materials/
        │   ├── models/
        │   ├── rtshaderlib/
        │   └── skyx/
        ├── models/              ← Our models
        │   ├── Base Station/
        │   ├── cardboard_box/
        │   ├── custom_ground_plane/
        │   ├── house_1/
        │   ├── turtlebot3_burger_1/
        │   ├── turtlebot3_burger_2/
        │   ├── turtlebot3_burger_3/
        |   └── turtlebot3_burger_4/
        ├── scripts/             ← Our scripts (for spawning)
        │   ├── spawn_houses.py
        │   ├── spawn_obstacles.py
        |   └── spawn_entities.py
        ├── config/              ← Our configuration file
        |   └── shared_settings.yaml
        ├── srv/                 ← Our services
        |   ├── relay/
        |   └── grid/
        └── src/                 ←  Python nodes here
            ├── output/
            ├── settings/
            ├── __init__.py
            ├── a_star.py
            ├── dispatcher.py
            ├── grid_manager.py
            ├── relay_server.py
            ├── robot_controller_node.py
            ├── robot.py
            └── world.py
~~~

