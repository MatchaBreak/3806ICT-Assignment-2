from my_sim_pkg.srv import UpdateCurrentBotPositionRequest
from collections import deque
from enum import Enum
from a_star import AStar
from world import Settings, TileType
import rospy

class RobotState(Enum):
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3

class Robot:
    def __init__(self, robot_id, controller, signal_queue, listen_order):
        self.id = robot_id
        self.controller = controller
        self.signal_queue = signal_queue
        self.listen_order = listen_order

        self.state = RobotState.QUEUING
        self.home_location = Settings.robot_positions[robot_id]
        self.current_location = self.home_location
        self.goals = []
        self.current_path = deque()
        self.current_goal = None

    def add_goal(self, goal):
        if goal not in self.goals:
            self.goals.append(goal)
            rospy.loginfo(f"ROBOT AGENT::{self.id} added goal: {goal}")
            if self.state == RobotState.QUEUING:
                self.state = RobotState.WAITING_FOR_ORDER

    def is_adjacent_to_goal(self):
        if self.current_goal is None:
            return False
        x, y = self.current_location
        gx, gy = self.current_goal
        return abs(x - gx) + abs(y - gy) == 1

    def plan_path(self, goal):
        a_star = AStar(self.current_location, goal, self.controller.latest_world)
        path = a_star.find_path()
        if path and len(path) > 1:
            self.current_path = deque(path[1:])
            self.current_goal = goal
            rospy.loginfo(f"ROBOT AGENT::{self.id} planned path to {goal}")
        else:
            rospy.logwarn(f"ROBOT AGENT::{self.id} could not find path to {goal}")

    def move(self):
        if not self.current_path:
            return

        if self.detect_obstacle_ahead():
            rospy.loginfo(f"ROBOT AGENT::{self.id} obstacle detected, replanning path.")
            if self.current_goal:
                self.plan_path(self.current_goal)
            return

        current_x, current_y = self.current_location
        next_x, next_y = self.current_path.popleft()
        dx, dy = next_x - current_x, next_y - current_y
        direction = self.controller.directions.get((dx, dy))

        if not direction:
            rospy.logwarn(f"ROBOT AGENT::{self.id} invalid move from {self.current_location} to {(next_x, next_y)}")
            return

        try:
            req = UpdateCurrentBotPositionRequest()
            req.botID = self.id
            req.direction = direction
            req.currentX = current_x
            req.currentY = current_y
            res = self.controller.update_pos(req)

            if res.success:
                self.controller.teleport_robot(self.id, next_x, next_y)
                self.current_location = (next_x, next_y)
                rospy.loginfo(f"ROBOT AGENT::{self.id} moved to {self.current_location}")
            else:
                rospy.logwarn(f"ROBOT AGENT::{self.id} position update blocked.")
                self.current_path.clear()

        except rospy.ServiceException as e:
            rospy.logerr(f"ROBOT AGENT::{self.id} service call failed: {e}")

    def update_state(self):
        if self.state == RobotState.QUEUING:
            try:
                res = self.signal_queue(botID=self.id)
                if res.success:
                    rospy.loginfo(f"Robot {self.id} queued successfully.")
                    self.state = RobotState.WAITING_FOR_ORDER
            except rospy.ServiceException:
                pass

        elif self.state == RobotState.WAITING_FOR_ORDER:
            try:
                res = self.listen_order(botId=self.id)
                if res.orderTaken:
                    goal = (res.deliveryLocationX, res.deliveryLocationY)
                    self.add_goal(goal)
                    self.plan_path(goal)
                    self.state = RobotState.DELIVERING
            except rospy.ServiceException:
                pass

        elif self.state in (RobotState.DELIVERING, RobotState.GOING_HOME):
            # Skip move if adjacent to the goal (for delivery)
            if self.state == RobotState.DELIVERING and self.is_adjacent_to_goal():
                rospy.loginfo(f"Robot {self.id} delivered adjacent to {self.current_goal}")
                self.plan_path(self.home_location)
                self.state = RobotState.GOING_HOME
                return

            self.move()

            # Check if robot reached home (only applies when GOING_HOME)
            if not self.current_path and self.current_location == self.current_goal:
                rospy.loginfo(f"Robot {self.id} returned home")
                self.state = RobotState.QUEUING
                self.current_goal = None


    def detect_obstacle_ahead(self):
        if not self.current_path:
            return False

        next_x, next_y = self.current_path[0]
        #rospy.loginfo(f"ROBOT AGENT::{self.id} checking tile ahead: ({next_x}, {next_y})")
        #rospy.loginfo(f"ROBOT AGENT::{self.id} current unknowns: {self.controller.unknown_obstacles}")

        if (next_x, next_y) in self.controller.unknown_obstacles:
            rospy.logwarn(f"ROBOT AGENT::{self.id} discovered obstacle at {(next_x, next_y)}")
            #this tells the controller to tell the grid manager to update the authoritative interpretation of the world
            self.controller.mark_obstacle(next_x, next_y)
            self.controller.unknown_obstacles.remove((next_x, next_y))
            self.controller.known_obstacles.append((next_x, next_y))
            self.current_path.clear()
            return True
        
        elif(next_x, next_y) in self.controller.known_obstacles:
            rospy.logwarn(f"ROBOT AGENT::{self.id} obstacle detected at {(next_x, next_y)}")
            self.current_path.clear()
            return True

        return False
