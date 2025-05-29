from collections import deque
from enum import Enum


class RobotState(Enum):
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3

class Robot:
    def __init__(self, bot_id, controller):
        self.id = bot_id
        self.controller = controller
        self.state = RobotState.QUEUING
        self.path = deque()
        self.location = Settings.robot_positions[bot_id]
        self.goal = None
        self.delivery = None

    def update(self):
        if self.state == RobotState.QUEUING:
            self.handle_queuing()
        elif self.state == RobotState.WAITING_FOR_ORDER:
            self.handle_waiting()
        elif self.state == RobotState.DELIVERING:
            self.handle_delivering()
        elif self.state == RobotState.GOING_HOME:
            self.handle_going_home()

    def plan_path(self, goal):
        a_star = AStar(self.location, goal, self.controller.latest_world)
        # Exclude other robots' next moves to avoid future collisions
        exclude = [r.path[0] for r in self.controller.robots.values() if r.id != self.id and r.path]
        a_star.set_blocked_tiles(exclude)
        path = a_star.find_path()
        if path:
            self.path = deque(path[1:])
            self.goal = goal

    def move(self):
        if self.path:
            next_x, next_y = self.path.popleft()
            self.controller.teleport_robot(self.id, next_x, next_y)
            self.location = (next_x, next_y)
