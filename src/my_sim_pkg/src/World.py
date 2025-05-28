import rospy

class TileType:
    UNVISITED = 0
    VISITED = -1
    OBSTACLE = -2
    PIZZA = -3
    HOUSE = -4
    NONEXIST = -1  # Invalid/out-of-bounds


class World:
    def __init__(self, width, height, tile_size=1):
        self.width = width if width is not None else rospy.get_param('GRID_WIDTH')
        self.height = height if height is not None else rospy.get_param('GRID_HEIGHT')
        self.tile_size = tile_size
        self.grid = [[TileType.UNVISITED for _ in range(self.width)] for _ in range(self.height)]

        home_x = rospy.get_param('HOME_X')
        home_y = rospy.get_param('HOME_Y')
        self.base_position = (home_x, home_y)
        self.grid[home_y][home_x] = TileType.PIZZA

    def set_tile(self, x, y, value):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = value

    def get_tile(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y][x]
        return TileType.NONEXIST

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_base_position(self):
        return self.base_position

    def get_world_as_string(self):
        lines = []
        for row in self.grid:
            line = ', '.join(str(cell) for cell in row)
            lines.append(f"[{line}]")
        return '\n'.join(lines)
