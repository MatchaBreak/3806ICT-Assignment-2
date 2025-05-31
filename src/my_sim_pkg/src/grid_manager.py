import rospy
from std_msgs.msg import Int32MultiArray
from gazebo_msgs.srv import GetModelState
from my_sim_pkg.srv import (
    UpdateCurrentBotPosition,
    UpdateCurrentBotPositionResponse,
    GetHouseLocations,
    GetHouseLocationsResponse,
)

#to let the controller update the world state
from my_sim_pkg.srv import UpdateGrid, UpdateGridResponse

# our files
from settings.load_yaml import load_shared_settings
from world import World, TileType
from robot import Robot


class GridManager:
    def __init__(self):
        self.world = World()
        
        self.settings = load_shared_settings()
        # Load shared settings using the helper function
        self.GRID_WIDTH = self.settings.get("GRID_WIDTH", 16)
        self.GRID_HEIGHT = self.settings.get("GRID_HEIGHT", 16)
        self.TILE_SIZE = self.settings.get("TILE_SIZE", 1)
        self.NUM_ROBOTS = self.settings.get("NUM_ROBOTS", 4)
        self.NUM_HOUSES = self.settings.get("NUM_HOUSES", 4)
       
        # So it knows how to move
        self.direction_map = {
            key: tuple(value) for key, value in self.settings.get("direction_map", {}).items()
        }
        """
            direction_map = {
                "moveLeft": (0, -1),
                "moveRight": (0, 1),
                "moveUp": (-1, 0),
                "moveDown": (1, 0),
            }
        """
        self.populate_houses()
        self.populate_robots()

        self.grid_pub = rospy.Publisher('/world_grid', Int32MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.publish_grid)

        rospy.Service('/update_current_bot_position', UpdateCurrentBotPosition, self.update_position)
        rospy.Service('/get_house_locations', GetHouseLocations, self.get_house_locations)
        # to let the controller update the world state
        rospy.Service('/update_grid', UpdateGrid, self.update_grid)
        rospy.loginfo("GRID MANAGER::Grid Manager ready.")
        self.world.dump_to_file()

    def update_grid(self, req):
        if self.world.in_bounds(req.tilePositionX, req.tilePositionY):
            self.world.set_tile(req.tilePositionX, req.tilePositionY, req.tileType)
            rospy.loginfo(f"GRID MANAGER::Updated tile at ({req.tilePositionX}, {req.tilePositionY}) to {req.tileType}")
            self.world.dump_to_file()
            return UpdateGridResponse(success=True)
        else:
            rospy.logwarn(f"GRID MANAGER::Invalid grid update request at ({req.tilePositionX}, {req.tilePositionY})")
            return UpdateGridResponse(success=False)

    def populate_houses(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i in range(1, self.NUM_HOUSES + 1):
            name = f"house_{i}"
            try:
                res = get_model(name, "world")
                x = int(res.pose.position.x // self.TILE_SIZE)
                y = int(res.pose.position.y // self.TILE_SIZE)
                self.world.set_tile(x, y, TileType.HOUSE)
                self.world.houses.append((x, y))
                rospy.loginfo(f"GRID MANAGER:: House {i} is at ({x}, {y})")
            except rospy.ServiceException as e:
                rospy.logerr(f"GRID MANAGER::Failed to get model state for {name}: {e}")

    def populate_robots(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i in range(self.NUM_ROBOTS):
            name = f"turtlebot3_burger_{i+1}"
            try:
                res = get_model(name, "world")
                y = max(0, min(self.GRID_WIDTH - 1, round(res.pose.position.x / self.TILE_SIZE)))
                x = max(0, min(self.GRID_HEIGHT - 1, round(res.pose.position.y / self.TILE_SIZE)))
                tile_type = getattr(TileType, f"ROBOT_{i+1}")
                self.world.set_tile(x, y, tile_type)
                self.world.robots[i + 1] = (x, y)
                #self.robot_objects[i + 1] = Robot(robot_id=i + 1, node_handle=None, pat_helper=None)
                #self.robot_objects[i + 1].currentLocation = (x, y)
                rospy.loginfo(f"GRID MANAGER::Robot {i+1} initialized at ({x}, {y})")
            except rospy.ServiceException as e:
                rospy.logerr(f"GRID MANAGER::Failed to get model state for {name}: {e}")

    def update_position(self, req):
        dx, dy = self.direction_map.get(req.direction, (0, 0))
        curr_x, curr_y = req.currentX, req.currentY
        new_x = curr_x + dx
        new_y = curr_y + dy

        if self.world.in_bounds(new_x, new_y) and self.world.get_tile(new_x, new_y) in [
            TileType.UNVISITED,
            TileType.VISITED,
            TileType.HOUSE,
            TileType.ROBOT_1,
            TileType.ROBOT_2,
            TileType.ROBOT_3,
            TileType.ROBOT_4,
        ]:
            self.world.set_tile(curr_x, curr_y, TileType.VISITED)
            tile_type = getattr(TileType, f"ROBOT_{req.botID}")
            self.world.set_tile(new_x, new_y, tile_type)
            self.world.robots[req.botID] = (new_x, new_y)
            #if req.botID in self.robot_objects:
                #self.robot_objects[req.botID].currentLocation = (new_x, new_y)
            #rospy.loginfo(f"GRID MANAGER::Bot {req.botID} moved to ({new_x}, {new_y})")
            self.world.dump_to_file(f"move_bot_{req.botID}.txt")
            return UpdateCurrentBotPositionResponse(success=True)
        else:
            rospy.logwarn(f"GRID MANAGER::Bot {req.botID} failed to move to ({new_x}, {new_y}) - cell occupied or out of bounds.")
            return UpdateCurrentBotPositionResponse(success=False)

    def publish_grid(self, event):
        flat_grid = [cell for row in self.world.grid for cell in row]
        msg = Int32MultiArray(data=flat_grid)
        self.grid_pub.publish(msg)

    def get_house_locations(self, req):
        flat = [coord for (x, y) in self.world.houses for coord in (x, y)]
        return GetHouseLocationsResponse(locations=Int32MultiArray(data=flat))

if __name__ == '__main__':
    rospy.init_node('grid_manager')
    GridManager()
    rospy.spin()
