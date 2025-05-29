#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
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

NUM_ROBOTS = Settings.NUM_ROBOTS
GRID_SIZE = Settings.GRID_SIZE

directions = {
    (0, -1): "moveLeft",
    (0, 1): "moveRight",
    (-1, 0): "moveUp",
    (1, 0): "moveDown",
}

robot_positions = {}
robot_states = {}
robot_paths = {}
robot_locations = {}
robot_deliveries = {}
robot_goals = {}
lock = Lock()

class RobotState:
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3

def odom_callback(msg, bot_id):
    with lock:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        grid_x = int(round(x / GRID_SIZE))
        grid_y = int(round(y / GRID_SIZE))
        robot_positions[bot_id] = (grid_x, grid_y)

def teleport_robot(bot_id, x, y, client):
    try:
        req = SetModelStateRequest()
        req.model_state.model_name = f"turtlebot3_burger_{bot_id}"
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = 0.1
        req.model_state.pose.orientation.w = 1.0
        client(req)
        rospy.loginfo(f"ROBOT_CONTROLLER::Teleported robot {bot_id} to ({x}, {y})")
    except Exception as e:
        rospy.logwarn(f"ROBOT_CONTROLLER::Teleport failed: {e}")


def move_robot(bot_id, update_pos_client, update_grid_client, set_model_client, world):
    if not robot_paths[bot_id]:
        if robot_states[bot_id] == RobotState.DELIVERING and bot_id not in robot_goals:
            home_goal = Settings.robot_positions[bot_id]
            current_x, current_y = robot_locations[bot_id]
            a_star = AStar((current_x, current_y), home_goal, world)
            path = a_star.find_path()
            robot_paths[bot_id] = deque(path[1:] if path else [])
            robot_goals[bot_id] = home_goal
            robot_states[bot_id] = RobotState.GOING_HOME
        return

    current_x, current_y = robot_locations[bot_id]
    next_x, next_y = robot_paths[bot_id].popleft()
    dx, dy = next_x - current_x, next_y - current_y

    direction = directions.get((dx, dy), None)
    if direction is None:
        rospy.logwarn(f"Invalid move direction from ({current_x},{current_y}) to ({next_x},{next_y})")
        return

    try:
        req = UpdateCurrentBotPosition._request_class()
        req.botID = bot_id
        req.direction = direction
        req.currentX = current_x
        req.currentY = current_y
        response = update_pos_client(req)

        if response.success:
            teleport_robot(bot_id, next_x, next_y, set_model_client)
            robot_locations[bot_id] = (next_x, next_y)

            if bot_id in robot_goals and (next_x, next_y) == robot_goals[bot_id]:
                rospy.loginfo(f"ROBOT_CONTROLLER::Robot {bot_id} reached its goal at {robot_goals[bot_id]}")
                if robot_states[bot_id] == RobotState.DELIVERING:
                    home_goal = Settings.robot_positions[bot_id]
                    a_star = AStar((next_x, next_y), home_goal, world)
                    path = a_star.find_path()
                    robot_paths[bot_id] = deque(path[1:] if path else [])
                    robot_goals[bot_id] = home_goal
                    robot_states[bot_id] = RobotState.GOING_HOME
                elif robot_states[bot_id] == RobotState.GOING_HOME:
                    rospy.loginfo(f"ROBOT_CONTROLLER::Robot {bot_id} returned home to {robot_goals[bot_id]}")
                    robot_states[bot_id] = RobotState.QUEUING
                    robot_goals.pop(bot_id, None)

        else:
            rospy.loginfo(f"ROBOT_CONTROLLER::Robot {bot_id} blocked at ({next_x},{next_y})")
            robot_paths[bot_id].clear()

    except rospy.ServiceException:
        rospy.logerr("ROBOT_CONTROLLER::Failed to call update_current_bot_position")


def controller_main():
    rospy.init_node('robot_controller_node')
    world = World()
    
    signal_queue = rospy.ServiceProxy('/signal_queued_up', SignalQueuedUp)
    listen_order = rospy.ServiceProxy('/listen_to_order_status', ListenToOrderStatus)
    update_pos = rospy.ServiceProxy('/update_current_bot_position', UpdateCurrentBotPosition)
    update_grid = rospy.ServiceProxy('/update_grid', UpdateGrid)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    for i in range(1, NUM_ROBOTS + 1):
        topic = f"/turtlebot3_burger_{i}/odom"
        rospy.Subscriber(topic, Odometry, odom_callback, i)
        robot_states[i] = RobotState.QUEUING
        robot_paths[i] = deque()

    rate = rospy.Rate(1)
    rospy.loginfo("Waiting for odometry...")

    while not rospy.is_shutdown():
        if len(robot_positions) == NUM_ROBOTS:
            for i in range(1, NUM_ROBOTS + 1):
                robot_locations[i] = robot_positions[i]
                rospy.loginfo(f"ROBOT CONTROLLER::Robot {i} initialised at {robot_locations[i]}")
            break
        else:
            rospy.loginfo("ROBOT CONTROLLER::discrepancy in provided robot positions and declared NUM_ROBOTS")
        rate.sleep()

    def handle_queuing(i):
        try:
            response = signal_queue(botID=i)
            if response.success:
                robot_states[i] = RobotState.WAITING_FOR_ORDER
        except rospy.ServiceException:
            pass

    def handle_waiting_for_order(i):
        try:
            response = listen_order(botId=i)
            if response.orderTaken:
                delivery_x, delivery_y = response.deliveryLocationX, response.deliveryLocationY
                start = robot_locations[i]
                goal = (delivery_x, delivery_y)
                a_star = AStar(start, goal, world)
                path = a_star.find_path()
                if path:
                    robot_paths[i] = deque(path[1:])
                    robot_deliveries[i] = goal
                    robot_goals[i] = goal
                    robot_states[i] = RobotState.DELIVERING
                    rospy.loginfo(f"ROBOT CONTROLLER::Robot {i} path: {list(robot_paths[i])}")
                else:
                    rospy.logwarn(f"ROBOT CONTROLLER::Robot {i} has no path to ({delivery_x}, {delivery_y})")
        except rospy.ServiceException:
            pass

    def handle_delivering(i):
        move_robot(i, update_pos, update_grid, set_model_state, world)
        if not robot_paths[i]:
            delivery_x, delivery_y = robot_deliveries[i]
            current_x, current_y = robot_locations[i]
            if (current_x, current_y) == (delivery_x, delivery_y):
                rospy.loginfo(f"ROBOT CONTROLLER::Robot {i} delivered to ({delivery_x}, {delivery_y})")
                a_star = AStar((current_x, current_y), Settings.robot_positions[i], world)
                path = a_star.find_path()
                robot_paths[i] = deque(path[1:] if path else [])
                robot_states[i] = RobotState.GOING_HOME
            else:
                rospy.logwarn(f"ROBOT CONTROLLER::Robot {i} failed to reach ({delivery_x}, {delivery_y})")

    def handle_going_home(i):
        move_robot(i, update_pos, update_grid, set_model_state, world)
        if not robot_paths[i]:
            robot_states[i] = RobotState.QUEUING

    state_handlers = {
        RobotState.QUEUING: handle_queuing,
        RobotState.WAITING_FOR_ORDER: handle_waiting_for_order,
        RobotState.DELIVERING: handle_delivering,
        RobotState.GOING_HOME: handle_going_home,
    }
   
    while not rospy.is_shutdown():
        for i in range(1, NUM_ROBOTS + 1):
            handler = state_handlers.get(robot_states[i])
            if handler:
                handler(i)
        rate.sleep()
    
    """
    # Test-only: assign fake path up up up, left left left
    for i in range(1, NUM_ROBOTS + 1):
        x, y = robot_locations[i]
        path = [(x-1, y), (x-2, y), (x-3, y), (x-3, y-1), (x-3, y-2), (x-3, y-3)]
        robot_paths[i] = deque(path)
        robot_states[i] = RobotState.DELIVERING

    while not rospy.is_shutdown():
        for i in range(1, NUM_ROBOTS + 1):
            move_robot(i, update_pos, update_grid, set_model_state)
        rate.sleep()
     """


if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        pass
