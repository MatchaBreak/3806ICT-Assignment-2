import rospy
import rospkg
from settings.load_yaml import load_shared_settings
from gazebo_msgs.srv import SpawnModel, GetModelState
from geometry_msgs.msg import Pose
import random
import math

class SpawnEntities:
    def __init__(self):
        rospy.init_node("spawn_entities")

        # Load settings from YAML
        self.settings = load_shared_settings()
        self.seed = self.settings.get("SEED", 1)
        
        random.seed(self.seed)
        
        self.num_houses = self.settings.get("NUM_HOUSES", 4)
        self.num_obstacles = self.settings.get("NUM_OBSTACLES", 10)
        self.grid_min = self.settings.get("GRID_MIN", 0)
        self.grid_max = self.settings.get("GRID_MAX", 15)
        self.min_separation = self.settings.get("MIN_SEPARATION", 2)
        self.min_distance_from_base = self.settings.get("MIN_DISTANCE_FROM_BASE", 4)
        self.base_position = (self.settings.get("HOME_X", 8), self.settings.get("HOME_Y", 8))
        self.house_model_path = f"{rospkg.RosPack().get_path('my_sim_pkg')}/models/house_1/model.sdf"
        self.base_station_model_path = f"{rospkg.RosPack().get_path('my_sim_pkg')}/models/Base Station/model.sdf"
        self.obstacle_model_path = f"{rospkg.RosPack().get_path('my_sim_pkg')}/models/cardboard_box/model.sdf"

        rospy.loginfo(f"SPAWN_ENTITIES::Loaded settings: {self.settings}")
        self.spawn_base_station()
        self.spawn_houses_and_obstacles()

    def validate_house_positions(self, house_positions):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i, (x, y) in enumerate(house_positions, start=1):
            house_name = f"house_{i}"
            while not rospy.is_shutdown():
                try:
                    res = get_model_state(house_name, "world")
                    if res.success:
                        rospy.loginfo(f"Confirmed house {house_name} spawned.")
                        break
                except rospy.ServiceException:
                    pass
                rospy.sleep(0.5)

    def generate_random_positions(self, num_positions, existing_positions=None, center=None, min_distance_from_center=0):
        existing_positions = existing_positions if existing_positions else []
        positions = existing_positions[:]
        while len(positions) < num_positions + len(existing_positions):
            x = random.randint(self.grid_min, self.grid_max)
            y = random.randint(self.grid_min, self.grid_max)

            # Check minimum distance from center (base position)
            if center and math.sqrt((x - center[0]) ** 2 + (y - center[1]) ** 2) < min_distance_from_center:
                continue

            # Check minimum separation from other positions
            if all(math.sqrt((x - px) ** 2 + (y - py) ** 2) >= self.min_separation for px, py in positions):
                positions.append((x, y))
        return positions[len(existing_positions):]

    def spawn_model(self, model_name, model_path, x, y, z=0):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            with open(model_path, 'r') as f:
                model_xml = f.read()
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            response = spawn_model(model_name, model_xml, "", pose, "world")
            if response.success:
                rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")
            else:
                rospy.logerr(f"Failed to spawn {model_name}: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed for {model_name}: {e}")

    def spawn_base_station(self):
        rospy.loginfo("SPAWN_ENTITIES::Spawning base station at home position.")
        self.spawn_model("BaseStation", self.base_station_model_path, self.base_position[0], self.base_position[1])

    def spawn_houses_and_obstacles(self):
        # Spawn houses
        house_positions = self.generate_random_positions(
            self.num_houses,
            center=self.base_position,
            min_distance_from_center=self.min_distance_from_base
        )
        for i, (x, y) in enumerate(house_positions, start=1):
            self.spawn_model(f"house_{i}", self.house_model_path, x, y)

        # Validate house positions
        self.validate_house_positions(house_positions)

        # Generate valid positions for obstacles
        obstacle_positions = self.generate_random_positions(
            self.num_obstacles,
            existing_positions=house_positions,
            center=self.base_position,
            min_distance_from_center=self.min_distance_from_base
        )
        for i, (x, y) in enumerate(obstacle_positions, start=1):
            self.spawn_model(f"cardboard_box_{i}", self.obstacle_model_path, x, y)

if __name__ == "__main__":
    SpawnEntities()