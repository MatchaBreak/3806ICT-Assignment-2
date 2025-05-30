#to spawn obstacles which turtlebot sensors can pick up#to spawn obstacles which turtlebot sensors can pick up

import random
import math
import rospy
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from my_sim_pkg.srv import (
    GetHouseLocations,
)


def generate_random_positions(num_objects, grid_min, grid_max, min_separation, existing_positions=None, center=(8, 8), min_distance_from_center=4):
    existing_positions = existing_positions if existing_positions else []  # Ensure existing_positions is a list
    positions = existing_positions[:]
    while len(positions) < num_objects + len(existing_positions):
        x = random.randint(grid_min, grid_max)
        y = random.randint(grid_min, grid_max)

        # Ensure the position is at least `min_distance_from_center` away from the center
        if math.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2) < min_distance_from_center:
            continue

        # Ensure no overlap with existing positions
        if all(math.sqrt((x - px) ** 2 + (y - py) ** 2) >= min_separation for px, py in positions):
            positions.append((x, y))
    return positions[len(existing_positions):]  # Return only the new positions


def spawn_obstacle(model_name, model_path, x, y, z=0):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(model_path, 'r') as f:
            model_xml = f.read()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")


if __name__ == "__main__":

    rospy.init_node("spawn_obstacles")

    # Get parameters from the launch file or use defaults (defaults to 4 if not provided in the world_simulation.launch)
    num_obstacles = rospy.get_param("~num_obstacles", 4)
    # Minimum grid value (defaults to -8 if not provided in the world_simulation.launch)
    grid_min = rospy.get_param("~grid_min", -8)
    # Maximum grid value (defaults to 8 if not provided in the world_simulation.launch)
    grid_max = rospy.get_param("~grid_max", 8)
    min_separation = rospy.get_param("~min_separation", 2)
    model_path = rospy.get_param("~model_path", "/root/3806ICT-Assignment-2/src/my_sim_pkg/models/cardboard_box/model.sdf")

    # Generates the random positions
    positions = generate_random_positions(num_obstacles, grid_min, grid_max, min_separation)

    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.loginfo("Gazebo services are now available.")

    # Spawn obstacles in Gazebo
    for i, (x, y) in enumerate(positions, start=1):
        spawn_obstacle(f"cardboard_box_{i}", model_path, x, y)

    # PRINT PYTHON PATH via sys
    rospy.loginfo(f"OBSTACLE SPAWNER::Python Path: {sys.path}")