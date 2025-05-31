# To spawn houses and the Base Station in Gazebo

import random
import math
import rospy
import sys
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def generate_random_positions(num_houses, grid_min, grid_max, min_separation, center=(8, 8), min_distance_from_center=6):
    positions = []
    while len(positions) < num_houses:
        # Generate random integers for x and y within the grid range
        x = random.randint(grid_min, grid_max)
        y = random.randint(grid_min, grid_max)

        # Ensure the position is at least `min_distance_from_center` away from the center
        if math.sqrt((x - center[0])**2 + (y - center[1])**2) < min_distance_from_center:
            continue

        # Ensure no overlap with existing positions
        if all(math.sqrt((x - px)**2 + (y - py)**2) >= min_separation for px, py in positions):
            positions.append((x, y))
    return positions

def spawn_model(model_name, model_path, x, y, z=0):
    """
    Generic function to spawn a model in Gazebo.
    """
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
    rospy.init_node("spawn_houses")

    # Get parameters from the launch file or use defaults
    num_houses = rospy.get_param("~num_houses", 4)
    grid_min = rospy.get_param("~grid_min", -8)
    grid_max = rospy.get_param("~grid_max", 8)
    min_separation = rospy.get_param("~min_separation", 2)
    house_model_path = rospy.get_param("~model_path", "/root/3806ICT-Assignment-2/src/my_sim_pkg/models/house_1/model.sdf")
    base_station_model_path = rospy.get_param("~base_station_model_path", "/root/ros_python/3806ICT-Assignment-2/src/my_sim_pkg/models/Base Station/model.sdf")

    # Generate random positions for houses
    positions = generate_random_positions(num_houses, grid_min, grid_max, min_separation)

    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.loginfo("Gazebo services are now available.")

    # Spawn houses in Gazebo
    for i, (x, y) in enumerate(positions, start=1):
        spawn_model(f"house_{i}", house_model_path, x, y)

    # Spawn Base Station at the center (8, 8)
    spawn_model("BaseStation", base_station_model_path, 8, 8)

    # Publish house positions to the ROS parameter server
    rospy.set_param("/house_positions", positions)
    rospy.loginfo(f"HOUSE SPAWNER::House positions: {positions}")