#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Int32MultiArray
from gazebo_msgs.srv import GetModelState
from my_sim_pkg.srv import (
    UpdateCurrentBotPosition,
    UpdateCurrentBotPositionResponse,
    GetHouseLocations,
    GetHouseLocationsResponse,
)


GRID_WIDTH = rospy.get_param('GRID_WIDTH', 16)
GRID_HEIGHT = rospy.get_param('GRID_HEIGHT', 16)
GRID_SIZE = rospy.get_param('GRID_SIZE', 1)
NUM_ROBOTS = rospy.get_param('NUM_ROBOTS', 4)
NUM_HOUSES = rospy.get_param('NUM_HOUSES', 4)

catkin_root = os.getenv("CATKIN_ROOT_DIRECTORY")
world_csp_path = f"{catkin_root}/src/my_sim_pkg/pat/world.csp"

TILE_UNVISITED = 0
TILE_VISITED = -1
TILE_OBSTACLE = -2
TILE_PIZZA = -3
TILE_HOUSE = -4

class GridManager:
    def __init__(self):
        self.grid = [[TILE_UNVISITED for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.houses = []
        self.robot_positions = [(0, 0)] * NUM_ROBOTS

        self.populate_houses()
        self.populate_robots()

        rospy.Service('/update_current_bot_position', UpdateCurrentBotPosition, self.update_position)
        rospy.Service('/get_house_locations', GetHouseLocations, self.get_house_locations)
        rospy.loginfo("GRID MANAGER::Grid Manager ready.")

    def populate_houses(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i in range(1, NUM_HOUSES + 1):
            name = f"house_{i}"
            try:
                res = get_model(name, "world")
                x = int(res.pose.position.x // GRID_SIZE)
                y = int(res.pose.position.y // GRID_SIZE)
                self.houses.append((x, y))
                self.grid[x][y] = TILE_HOUSE
                rospy.loginfo(f"GRID MANAGER:: House {i} is at ({x}, {y})")
            except rospy.ServiceException as e:
                rospy.logerr(f"GRID MANAGER::Failed to get model state for {name}: {e}")

    def populate_robots(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i in range(NUM_ROBOTS):
            name = f"turtlebot3_burger_{i+1}"
            try:
                res = get_model(name, "world")
                #defensive clamping to ensure coordinates are within grid bounds
                x = max(0, min(GRID_WIDTH - 1, round(res.pose.position.x / GRID_SIZE)))
                y = max(0, min(GRID_HEIGHT - 1, round(res.pose.position.y / GRID_SIZE)))

                self.robot_positions[i] = (x, y)
                self.grid[y][x] = i +1
                rospy.loginfo(f"GRID MANAGER::Robot {i+1} initialized at ({x}, {y})")
            except rospy.ServiceException as e:
                rospy.logerr(f"GRID MANAGER::Failed to get model state for {name}: {e}")

    def get_house_locations(self, req):
        flat = [coord for house in self.houses for coord in house]
        response = GetHouseLocationsResponse()
        response.locations = Int32MultiArray(data=flat)
        self.write_pat_file()
        return response

    def update_position(self, req):
        direction_map = {
            "moveLeft": (0, -1),
            "moveRight": (0, 1),
            "moveUp": (-1, 0),
            "moveDown": (1, 0),
        }
        dx, dy = direction_map.get(req.direction, (0, 0))

        # row = y, col = x
        curr_row, curr_col = req.currentY, req.currentX
        new_row = curr_row + dy
        new_col = curr_col + dx

        if (0 <= new_row < GRID_HEIGHT and 0 <= new_col < GRID_WIDTH and
            self.grid[new_row][new_col] in [TILE_UNVISITED, TILE_VISITED]):
            self.grid[curr_row][curr_col] = TILE_VISITED
            self.grid[new_row][new_col] = req.botID
            self.robot_positions[req.botID - 1] = (new_row, new_col)
            self.write_pat_file()
            rospy.loginfo(f"GRID MANAGER::Bot {req.botID} moved to ({new_col}, {new_row})")
            return UpdateCurrentBotPositionResponse(success=True)
        else:
            rospy.logwarn(f"GRID MANAGER::Bot {req.botID} failed to move to ({new_col}, {new_row}) - cell occupied or out of bounds.")
            return UpdateCurrentBotPositionResponse(success=False)




    def write_pat_file(self):
        try:
            with open(world_csp_path, 'w') as f:
                f.write("#define robot1 1;\n#define robot2 2;\n#define robot3 3;\n#define robot4 4;\n\n")
                f.write(f"#define rows {GRID_HEIGHT};\n#define cols {GRID_WIDTH};\n\n")
                f.write("#define maxPizzasPerRobot 1;\n\n")
                f.write(f"#define unvisited {TILE_UNVISITED};\n#define visited {TILE_VISITED};\n")
                f.write(f"#define obstacle {TILE_OBSTACLE};\n#define pizza {TILE_PIZZA};\n#define house {TILE_HOUSE};\n\n")
                f.write("#define HOME_X 8;\n#define HOME_Y 8;\n\n")
                f.write("var pizzasToDeliver = 4;\n\n")
                f.write("var world[rows][cols]:{-4..4} = [\n")
                for idx, row in enumerate(self.grid):
                    line = ", ".join(str(cell) for cell in row)
                    if idx < len(self.grid) - 1:
                        f.write(line + ",\n")
                    else:
                        f.write(line + "\n")  # No trailing comma on the last line
                f.write("];\n\n")


                for i, (x, y) in enumerate(self.robot_positions, 1):
                    f.write(f"var r{i}X = {x};\n")  # Row index
                    f.write(f"var r{i}Y = {y};\n\n")  # Column index

                for i, (x, y) in enumerate(self.houses[:NUM_ROBOTS], 1):
                    f.write(f"var r{i}deliveryX = {x};\n")
                    f.write(f"var r{i}deliveryY = {y};\n\n")

            rospy.loginfo("GRID MANAGER::Updated PAT world.csp file.")
        except Exception as e:
            rospy.logerr(f"GRID MANAGER::Failed to write to PAT file: {e}")

if __name__ == '__main__':
    rospy.init_node('grid_manager')
    GridManager()
    rospy.spin()
