import yaml
import rospkg

def load_shared_settings():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("my_sim_pkg")  # Replace with your package name
    yaml_file_path = f"{package_path}/config/shared_settings.yaml"

    try:
        with open(yaml_file_path, "r") as yaml_file:
            settings = yaml.safe_load(yaml_file)
            return settings
    except Exception as e:
        rospy.logerr(f"Failed to load shared settings: {e}")
        return {}