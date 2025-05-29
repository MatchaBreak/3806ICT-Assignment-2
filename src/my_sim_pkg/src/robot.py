from collections import deque
from enum import Enum


class RobotState(Enum):
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3


class Robot:
    def __init__(self, robot_id, node_handle, pat_helper):
        self.id = robot_id
        self.currentState = RobotState.QUEUING
        self.currentLocation = (0, 0)  # (x, y)
        self.deliveryLocation = (0, 0)
        self.patPath = deque()
        self.pat_helper = pat_helper
        self.nh = node_handle

    def call_pat(self):
        if self.currentState == RobotState.DELIVERING:
            self.pat_helper.update_destination_position(
                self.currentLocation[0], self.currentLocation[1],
                self.deliveryLocation[0], self.deliveryLocation[1]
            )
            self.patPath = deque(self.pat_helper.get_path("delivering"))
            return True

        if self.currentState == RobotState.GOING_HOME:
            self.patPath = deque(self.pat_helper.get_path("home"))
            return True

        return False
