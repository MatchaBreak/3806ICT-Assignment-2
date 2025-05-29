from my_sim_pkg.srv import UpdateCurrentBotPositionRequest
from collections import deque
from enum import Enum
from a_star import AStar
from world import Settings
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

    def add_goal(self, goal):  # Ensure this line is indented correctly
        if goal not in self.goals:
            self.goals.append(goal)
            rospy.loginfo(f"ROBOT AGENT::{self.id} added goal: {goal}")
            if self.state == RobotState.QUEUING:
                self.state = RobotState.WAITING_FOR_ORDER

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

        current_x, current_y = self.current_location
        next_x, next_y = self.current_path.popleft()
        dx, dy = next_x - current_x, next_y - current_y
        direction = self.controller.directions.get((dx, dy))

        if not direction:
            rospy.logwarn(f"ROBOT AGENT::{self.id} invalid move from {self.current_location} to {(next_x, next_y)}")
            return

        try:
            # Create a request object for the service
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
            self.move()
            if not self.current_path:
                if self.current_location == self.current_goal:
                    if self.state == RobotState.DELIVERING:
                        rospy.loginfo(f"Robot {self.id} delivered to {self.current_goal}")
                        self.plan_path(self.home_location)
                        self.state = RobotState.GOING_HOME
                    else:
                        rospy.loginfo(f"Robot {self.id} returned home")
                        self.state = RobotState.QUEUING
                        self.current_goal = None
