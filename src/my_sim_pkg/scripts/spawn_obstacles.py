import rospy
import rospkg  
from settings.load_yaml import load_shared_settings
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import random
import math

class SpawnObstacles:
    def __init__(self):
        rospy.init_node("spawn_obstacles")

        # Set a fixed seed for reproducibility
        random.seed(42)

        # Load settings from YAML
        self.settings = load_shared_settings()
        self.num_obstacles = self.settings.get("NUM_OBSTACLES", 10)
        self.grid_min = self.settings.get("GRID_MIN", 0)
        self.grid_max = self.settings.get("GRID_MAX", 15)
        self.min_separation = self.settings.get("MIN_SEPARATION", 2)
        self.min_distance_from_base = self.settings.get("MIN_DISTANCE_FROM_BASE", 4)  # Minimum distance from base
        self.base_position = (self.settings.get("HOME_X", 8), self.settings.get("HOME_Y", 8))
        self.model_path = f"{rospkg.RosPack().get_path('my_sim_pkg')}/models/cardboard_box/model.sdf"

        rospy.loginfo(f"SPAWN_OBSTACLES::Loaded settings: {self.settings}")
        self.spawn_obstacles()

    def generate_random_positions(self, existing_positions=None):
        existing_positions = existing_positions if existing_positions else []
        positions = existing_positions[:]
        while len(positions) < self.num_obstacles + len(existing_positions):
            x = random.randint(self.grid_min, self.grid_max)
            y = random.randint(self.grid_min, self.grid_max)

            # Check minimum separation and minimum distance from base
            if all(math.sqrt((x - px) ** 2 + (y - py) ** 2) >= self.min_separation for px, py in positions) and \
               math.sqrt((x - self.base_position[0]) ** 2 + (y - self.base_position[1]) ** 2) >= self.min_distance_from_base:
                positions.append((x, y))
        return positions[len(existing_positions):]

    def spawn_obstacle(self, model_name, x, y, z=0):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            with open(self.model_path, 'r') as f:
                model_xml = f.read()
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            spawn_model(model_name, model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {model_name}: {e}")

    def spawn_obstacles(self):
        house_positions = []  # Load house positions dynamically if needed
        positions = self.generate_random_positions(existing_positions=house_positions)
        for i, (x, y) in enumerate(positions, start=1):
            self.spawn_obstacle(f"cardboard_box_{i}", x, y)

if __name__ == "__main__":
    SpawnObstacles()