import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Int32MultiArray
from gazebo_msgs.srv import SetModelStateRequest
from my_sim_pkg.srv import (
    ListenToOrderStatus,
    SignalQueuedUp,
    UpdateCurrentBotPosition,
    UpdateGrid,
)
from a_star import AStar
from collections import deque
from threading import Lock
from world import Settings, World

class RobotState:
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller_node')
        self.NUM_ROBOTS = Settings.NUM_ROBOTS
        self.GRID_SIZE = Settings.GRID_SIZE
        self.latest_world = World()
        self.lock = Lock()

        self.robot_positions = {}
        self.robot_states = {}
        self.robot_paths = {}
        self.robot_locations = {}
        self.robot_deliveries = {}
        self.robot_goals = {}

        self.directions = {
            (0, -1): "moveLeft",
            (0, 1): "moveRight",
            (-1, 0): "moveUp",
            (1, 0): "moveDown",
        }

        self.signal_queue = rospy.ServiceProxy('/signal_queued_up', SignalQueuedUp)
        self.listen_order = rospy.ServiceProxy('/listen_to_order_status', ListenToOrderStatus)
        self.update_pos = rospy.ServiceProxy('/update_current_bot_position', UpdateCurrentBotPosition)
        self.update_grid = rospy.ServiceProxy('/update_grid', UpdateGrid)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.Subscriber('/world_grid', Int32MultiArray, self.world_callback)

        for i in range(1, self.NUM_ROBOTS + 1):
            topic = f"/turtlebot3_burger_{i}/odom"
            rospy.Subscriber(topic, Odometry, self.odom_callback, i)
            self.robot_states[i] = RobotState.QUEUING
            self.robot_paths[i] = deque()

        self.wait_for_odometry()
        self.controller_loop()

    def odom_callback(self, msg, bot_id):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            grid_x = int(round(x / self.GRID_SIZE))
            grid_y = int(round(y / self.GRID_SIZE))
            self.robot_positions[bot_id] = (grid_x, grid_y)

    def teleport_robot(self, bot_id, x, y, steps=10, delay=0.05):
        current_x, current_y = self.robot_locations.get(bot_id, (x, y))
        for i in range(1, steps + 1):
            interp_x = current_x + (x - current_x) * (i / steps)
            interp_y = current_y + (y - current_y) * (i / steps)
            try:
                req = SetModelStateRequest()
                req.model_state.model_name = f"turtlebot3_burger_{bot_id}"
                req.model_state.pose.position.x = interp_x
                req.model_state.pose.position.y = interp_y
                req.model_state.pose.position.z = 0.1
                req.model_state.pose.orientation.w = 1.0
                self.set_model_state(req)
            except Exception as e:
                rospy.logwarn(f"ROBOT CONTROLLER::Smooth teleport failed: {e}")
                break
            rospy.sleep(delay)
        rospy.loginfo(f"ROBOT CONTROLLER::Smoothly moved robot {bot_id} to ({x}, {y})")


    def world_callback(self, msg):
        data = msg.data
        width = self.latest_world.width
        height = self.latest_world.height
        for i in range(height):
            for j in range(width):
                idx = i * width + j
                self.latest_world.grid[i][j] = data[idx]

    def wait_for_odometry(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if len(self.robot_positions) == self.NUM_ROBOTS:
                for i in range(1, self.NUM_ROBOTS + 1):
                    self.robot_locations[i] = self.robot_positions[i]
                    rospy.loginfo(f"ROBOT CONTROLLER::Robot {i} initialised at {self.robot_locations[i]}")
                break
            rate.sleep()

    def move_robot(self, bot_id):
        if not self.robot_paths[bot_id]:
            if self.robot_states[bot_id] == RobotState.DELIVERING and bot_id not in self.robot_goals:
                home_goal = Settings.robot_positions[bot_id]
                current_x, current_y = self.robot_locations[bot_id]
                a_star = AStar((current_x, current_y), home_goal, self.latest_world)
                path = a_star.find_path()
                self.robot_paths[bot_id] = deque(path[1:] if path else [])
                self.robot_goals[bot_id] = home_goal
                self.robot_states[bot_id] = RobotState.GOING_HOME
            return

        current_x, current_y = self.robot_locations[bot_id]
        next_x, next_y = self.robot_paths[bot_id].popleft()
        dx, dy = next_x - current_x, next_y - current_y

        direction = self.directions.get((dx, dy), None)
        if direction is None:
            rospy.logwarn(f"ROBOT CONTROLLER::Invalid move direction from ({current_x},{current_y}) to ({next_x},{next_y})")
            return

        try:
            req = UpdateCurrentBotPosition._request_class()
            req.botID = bot_id
            req.direction = direction
            req.currentX = current_x
            req.currentY = current_y
            response = self.update_pos(req)

            if response.success:
                self.teleport_robot(bot_id, next_x, next_y)
                self.robot_locations[bot_id] = (next_x, next_y)

                if bot_id in self.robot_goals and (next_x, next_y) == self.robot_goals[bot_id]:
                    rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} reached its goal at {self.robot_goals[bot_id]}")
                    if self.robot_states[bot_id] == RobotState.DELIVERING:
                        home_goal = Settings.robot_positions[bot_id]
                        a_star = AStar((next_x, next_y), home_goal, self.latest_world)
                        path = a_star.find_path()
                        self.robot_paths[bot_id] = deque(path[1:] if path else [])
                        self.robot_goals[bot_id] = home_goal
                        self.robot_states[bot_id] = RobotState.GOING_HOME
                    elif self.robot_states[bot_id] == RobotState.GOING_HOME:
                        rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} returned home")
                        self.robot_states[bot_id] = RobotState.QUEUING
                        self.robot_goals.pop(bot_id, None)
            else:
                rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} blocked at ({next_x},{next_y})")
                self.robot_paths[bot_id].clear()
        except rospy.ServiceException:
            rospy.logerr("ROBOT CONTROLLER::Failed to call update_current_bot_position")

    def handle_queuing(self, bot_id):
        try:
            response = self.signal_queue(botID=bot_id)
            if response.success:
                self.robot_states[bot_id] = RobotState.WAITING_FOR_ORDER
        except rospy.ServiceException:
            pass

    def handle_waiting_for_order(self, bot_id):
        try:
            response = self.listen_order(botId=bot_id)
            if response.orderTaken:
                delivery_x, delivery_y = response.deliveryLocationX, response.deliveryLocationY
                start = self.robot_locations[bot_id]
                goal = (delivery_x, delivery_y)
                a_star = AStar(start, goal, self.latest_world)
                path = a_star.find_path()
                if path:
                    self.robot_paths[bot_id] = deque(path[1:])
                    self.robot_deliveries[bot_id] = goal
                    self.robot_goals[bot_id] = goal
                    self.robot_states[bot_id] = RobotState.DELIVERING
                    rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} path: {list(self.robot_paths[bot_id])}")
                else:
                    rospy.logwarn(f"ROBOT CONTROLLER::No path to delivery location for robot {bot_id}")
        except rospy.ServiceException:
            pass

    def handle_delivering(self, bot_id):
        self.move_robot(bot_id)
        if not self.robot_paths[bot_id]:
            delivery = self.robot_deliveries[bot_id]
            current = self.robot_locations[bot_id]
            if current == delivery:
                rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} delivered to {delivery}")
                a_star = AStar(current, Settings.robot_positions[bot_id], self.latest_world)
                path = a_star.find_path()
                self.robot_paths[bot_id] = deque(path[1:] if path else [])
                self.robot_states[bot_id] = RobotState.GOING_HOME
            else:
                rospy.logwarn(f"ROBOT CONTROLLER::Robot {bot_id} failed to reach delivery point")

    def handle_going_home(self, bot_id):
        self.move_robot(bot_id)
        if not self.robot_paths[bot_id]:
            self.robot_states[bot_id] = RobotState.QUEUING

    def controller_loop(self):
        rate = rospy.Rate(1)
        state_handlers = {
            RobotState.QUEUING: self.handle_queuing,
            RobotState.WAITING_FOR_ORDER: self.handle_waiting_for_order,
            RobotState.DELIVERING: self.handle_delivering,
            RobotState.GOING_HOME: self.handle_going_home,
        }
        while not rospy.is_shutdown():
            for i in range(1, self.NUM_ROBOTS + 1):
                handler = state_handlers.get(self.robot_states[i])
                if handler:
                    handler(i)
            rate.sleep()

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
