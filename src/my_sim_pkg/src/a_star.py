from world import TileType

class AStar:
    def __init__(self, start, goal, world):
        self.start = start
        self.goal = goal
        self.world = world
        self.open_set = set()
        self.closed_set = set()
        self.g_score = {}
        self.f_score = {}
        self.came_from = {}
        
    # Manhattan distance heuristic
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_path(self):
        self.open_set.add(self.start)
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.heuristic(self.start, self.goal)

        while self.open_set:
            current = min(self.open_set, key=lambda x: self.f_score.get(x, float('inf')))
            if current == self.goal:
                return self.reconstruct_path(current)

            self.open_set.remove(current)
            self.closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_score[current] + 1

                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                elif tentative_g_score >= self.g_score.get(neighbor, float('inf')):
                    continue

                self.came_from[neighbor] = current
                self.g_score[neighbor] = tentative_g_score
                self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)

        return None

    def reconstruct_path(self, current):
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        return path[::-1]

    # Allows for delivering to other bots (in theory)
    def get_neighbors(self, node):
        row, col = node
        neighbors = []

        for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            n_row, n_col = row + d_row, col + d_col
            if self.world.in_bounds(n_row, n_col):
                tile = self.world.get_tile(n_row, n_col)
                if tile in [TileType.UNVISITED, TileType.VISITED, TileType.HOUSE]:
                    neighbors.append((n_row, n_col))
                elif tile in [TileType.ROBOT_1, TileType.ROBOT_2, TileType.ROBOT_3, TileType.ROBOT_4]:
                    # Allow moving into robot-tile only if it's the goal (e.g., for rendezvous)
                    if (n_row, n_col) == self.goal:
                        neighbors.append((n_row, n_col))

        return neighbors

