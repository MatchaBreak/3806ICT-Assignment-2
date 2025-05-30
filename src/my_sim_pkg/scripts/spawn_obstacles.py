# To spawn obstacles which TurtleBot sensors can detect

import random
import math
import rospy
import sys
from gazebo_msgs.srv import SpawnModel, GetModelState
from geometry_msgs.msg import Pose

def generate_random_positions(num_objects, grid_min, grid_max, min_separation,
                               existing_positions=None, center=(8, 8), min_distance_from_center=4):
    existing_positions = existing_positions if existing_positions else []
    positions = existing_positions[:]
    while len(positions) < num_objects + len(existing_positions):
        x = random.randint(grid_min, grid_max)
        y = random.randint(grid_min, grid_max)

        if math.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2) < min_distance_from_center:
            continue

        if all(math.sqrt((x - px) ** 2 + (y - py) ** 2) >= min_separation for px, py in positions):
            positions.append((x, y))
    return positions[len(existing_positions):]


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

    num_obstacles = rospy.get_param("~num_obstacles", 4)
    grid_min = rospy.get_param("~grid_min", -8)
    grid_max = rospy.get_param("~grid_max", 8)
    min_separation = rospy.get_param("~min_separation", 2)
    model_path = rospy.get_param("~model_path", "/root/3806ICT-Assignment-2/src/my_sim_pkg/models/cardboard_box/model.sdf")

    # Load house positions from parameter server
    house_positions = rospy.get_param("/house_positions", [])
    house_positions = [tuple(pos) for pos in house_positions]

    # Wait for all house models to spawn in Gazebo
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    for i in range(len(house_positions)):
        house_name = f"house_{i+1}"
        while not rospy.is_shutdown():
            try:
                res = get_model_state(house_name, "world")
                if res.success:
                    rospy.loginfo(f"Confirmed house {house_name} spawned.")
                    break
            except rospy.ServiceException:
                pass
            rospy.sleep(0.5)

    # Generate valid positions for obstacles
    obstacle_positions = generate_random_positions(
        num_obstacles,
        grid_min,
        grid_max,
        min_separation,
        existing_positions=house_positions
    )

    rospy.loginfo("Waiting for Gazebo spawn model service...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.loginfo("Gazebo spawn model service available.")

    for i, (x, y) in enumerate(obstacle_positions, start=1):
        spawn_obstacle(f"cardboard_box_{i}", model_path, x, y)
