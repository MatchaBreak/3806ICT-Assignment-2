import rospy
import os
from settings.load_yaml import load_shared_settings


"""
    direction_map = {
        "moveLeft": (0, -1),
        "moveRight": (0, 1),
        "moveUp": (-1, 0),
        "moveDown": (1, 0),
    }
"""

class TileType:
    UNVISITED = 0
    ROBOT_1 = 1
    ROBOT_2 = 2
    ROBOT_3 = 3
    ROBOT_4 = 4
    VISITED = -1
    OBSTACLE = -2
    PIZZA = -3
    HOUSE = -4
    NONEXIST = -1  # Invalid/out-of-bounds


class World:
    def __init__(self):
        self.settings = load_shared_settings()
        # Grid settings
        self.width = self.settings.get("GRID_WIDTH", 16)
        self.height = self.settings.get("GRID_HEIGHT", 16)
        self.tile_size = self.settings.get("TILE_SIZE", 1)
        
        # Base position
        self.base_position = (
            self.settings.get("HOME_X", 8),
            self.settings.get("HOME_Y", 8)
        )
        # Initialise grid
        self.grid = [[TileType.UNVISITED for _ in range(self.width)] for _ in range(self.height)]
        self.grid[self.base_position[1]][self.base_position[0]] = TileType.PIZZA

        # Robot positions
        self.robot_positions = {
            i: (
                self.settings.get(f"robot_{i}_x", 0),
                self.settings.get(f"robot_{i}_y", 0)
            )
            for i in range(1, self.settings.get("NUM_ROBOTS", 4) + 1)
        }

        self.houses = []
        self.robots = {}


    def set_tile(self, x, y, value):
        if 0 <= x < self.width and 0 <= y < self.height:
            if value == TileType.OBSTACLE:
                print(f"\033[91m\033[1mSET TILE AS OBSTACLE AT ({x}, {y})\033[0m")
            self.grid[x][y] = value


    def get_tile(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[x][y]
        return TileType.NONEXIST

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def get_base_position(self):
        return self.base_position

    def get_world_as_string(self):
        lines = []
        lines.append(f"#define rows {self.height};")
        lines.append(f"#define cols {self.width};")
        lines.append("#define maxPizzasPerRobot 1;")
        lines.append(f"#define unvisited {TileType.UNVISITED};")
        lines.append(f"#define visited {TileType.VISITED};")
        lines.append(f"#define obstacle {TileType.OBSTACLE};")
        lines.append(f"#define pizza {TileType.PIZZA};")
        lines.append(f"#define house {TileType.HOUSE};")
        lines.append(f"#define HOME_X {self.base_position[0]};")
        lines.append(f"#define HOME_Y {self.base_position[1]};")
        lines.append("var pizzasToDeliver = 4;\n")
        lines.append("var world[rows][cols]:{-4..4} = [")
        for idx, row in enumerate(self.grid):
            row_str = ", ".join(f"{cell:2}" for cell in row)
            if idx < len(self.grid) - 1:
                lines.append(f"{row_str},")
            else:
                lines.append(f"{row_str}")
        lines.append("];")
        return '\n'.join(lines)

    def dump_to_file(self, filename="world_snapshot.txt"):
        output_dir = os.path.join(os.path.dirname(__file__), "output")
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        try:
            with open(filepath, "w") as f:
                f.write(self.get_world_as_string())
            rospy.loginfo(f"World state written to {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write world state: {e}")
