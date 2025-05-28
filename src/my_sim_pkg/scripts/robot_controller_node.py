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
    GetHouseLocations,
)

import time

#from python_pkgs.pat_helper import PATHelper


from collections import deque
from threading import Lock
import os
import subprocess

class PATHelper:
    def __init__(self, robot_id):
        self.robot_id = robot_id

        home = os.environ.get("HOME")
        catkin_root = os.environ.get("CATKIN_ROOT_DIRECTORY")
        pat_root = os.environ.get("PAT_ROOT_DIRECTORY")
        src = f"{catkin_root}/src/my_sim_pkg/pat"
        robot_path = os.path.join(src, f"robot{robot_id}")

        self.delivery_output = os.path.join(robot_path, "pizzaDeliveryOutput.txt")  # Match C++ version
        self.go_home_output = os.path.join(robot_path, "goHomeOutput.txt")
        self.deliver_csp = os.path.join(robot_path, f"robot{robot_id}deliverpizza.csp")
        self.gohome_csp = os.path.join(robot_path, f"robot{robot_id}gohome.csp")
        self.pat_exec = os.path.join(pat_root, "PAT3.Console.exe")
        self.bfs_timeout = 10  # seconds
        self.world_csp_path = os.path.join(src, "world.csp")

    def get_path(self, goal, delivery_x=None, delivery_y=None):
        if goal == "delivering":
            rospy.loginfo(f"ROBOT CONTROLLER: Robot {self.robot_id} TRYING TO deliver pizza to ({delivery_x}, {delivery_y})")
            if delivery_x is None or delivery_y is None:
                raise ValueError("Missing delivery coordinates for delivery goal.")

            cmd = f"mono {self.pat_exec} -engine 1 {self.deliver_csp} {self.delivery_output}"
            output_file = self.delivery_output
        elif goal == "home":
            cmd = f"mono {self.pat_exec} {self.gohome_csp} {self.go_home_output}"
            output_file = self.go_home_output
        else:
            raise ValueError("Invalid goal")

        time.sleep(0.2)  # Ensure file writes are flushed before PAT reads

        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        rospy.loginfo(f"PAT STDOUT: {result.stdout}")
        rospy.loginfo(f"PAT STDERR: {result.stderr}")

        path = []
        if os.path.exists(output_file):
            with open(output_file, "r") as f:
                for line in f:
                    if line.startswith("<"):
                        moves = line.split("->")[1:]
                        for m in moves:
                            move = m.strip().split()[0]
                            clean_move = move.replace(">", "").strip()
                            for direction in directions:
                                if clean_move.startswith(direction):
                                    path.append(direction)
                                    break

        if path:
            path_str = " -> ".join(path)
            rospy.loginfo(f"ROBOT CONTROLLER: Robot {self.robot_id} path to {goal}: {path_str}")
        else:
            rospy.loginfo(f"ROBOT CONTROLLER: *ALERT* Robot {self.robot_id} has no path to {goal} *ALERT*")

        return path


    def update_world_csp(self, current_x, current_y, delivery_x, delivery_y):
        try:
            with open(self.world_csp_path, 'r') as f:
                lines = f.readlines()

            new_lines = []
            for line in lines:
                if f"r{self.robot_id}X" in line:
                    new_lines.append(f"var r{self.robot_id}X:{0..rows-1} = {current_x};\n")
                elif f"r{self.robot_id}Y" in line:
                    new_lines.append(f"var r{self.robot_id}Y:{0..cols-1} = {current_y};\n")
                elif f"r{self.robot_id}deliveryX" in line:
                    new_lines.append(f"var r{self.robot_id}deliveryX = {delivery_x};\n")
                elif f"r{self.robot_id}deliveryY" in line:
                    new_lines.append(f"var r{self.robot_id}deliveryY = {delivery_y};\n")
                else:
                    new_lines.append(line)

            with open(self.world_csp_path, 'w') as f:
                f.writelines(new_lines)
                f.flush()
                os.fsync(f.fileno())  # Ensure all data is written to disk

            rospy.loginfo(f"PATHelper: Updated world.csp for robot {self.robot_id}")

        except Exception as e:
            rospy.logerr(f"PATHelper: Failed to update world.csp for robot {self.robot_id}: {e}")
# Constants
NUM_ROBOTS = rospy.get_param('NUM_ROBOTS')
GRID_SIZE = rospy.get_param('GRID_SIZE')

# Robot state enum
class RobotState:
    QUEUING = 0
    WAITING_FOR_ORDER = 1
    DELIVERING = 2
    GOING_HOME = 3

# Position and direction
directions = {
    "moveLeft": (0, -1),
    "moveRight": (0, 1),
    "moveUp": (-1, 0),
    "moveDown": (1, 0),
}

# Global data
robot_positions = {}
robot_states = {}
robot_paths = {}
robot_locations = {}
robot_deliveries = {}
robot_helpers = {}
lock = Lock()

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
        req.model_state.pose.position.x = y
        req.model_state.pose.position.y = x
        req.model_state.pose.position.z = 0.1
        req.model_state.pose.orientation.w = 1.0
        client(req)
        rospy.loginfo(f"ROBOT_CONTROLLER::Teleported robot {bot_id} to ({x}, {y})")
    except Exception as e:
        rospy.logwarn(f"ROBOT_CONTROLLER::Teleport failed: {e}")

def move_robot(bot_id, update_pos_client, update_grid_client, set_model_client):
    if not robot_paths[bot_id]:
        return

    move = robot_paths[bot_id][0]
    dx, dy = directions[move]
    current_x, current_y = robot_locations[bot_id]
    new_x, new_y = current_x + dx, current_y + dy

    try:
        req = UpdateCurrentBotPosition._request_class()
        req.botID = bot_id
        req.direction = move
        req.currentX = current_x
        req.currentY = current_y
        response = update_pos_client(req)

        if response.success:
            teleport_robot(bot_id, new_x, new_y, set_model_client)
            robot_locations[bot_id] = (new_x, new_y)
            robot_paths[bot_id].popleft()
        else:
            rospy.loginfo(f"ROBOT_CONTROLLER::Robot {bot_id} blocked at ({new_x},{new_y})")
            robot_paths[bot_id].clear()  # Clear path to trigger re-planning

    except rospy.ServiceException:
        rospy.logerr("ROBOT_CONTROLLER::Failed to call update_current_bot_position")

def controller_main():
    rospy.init_node('robot_controller_node')

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
        robot_deliveries[i] = (0, 0)
        robot_helpers[i] = PATHelper(i)

    rate = rospy.Rate(1) # one update per second
    rospy.loginfo("Waiting for odometry...")

    while not rospy.is_shutdown():
        if len(robot_positions) == NUM_ROBOTS:
            for i in range(1, NUM_ROBOTS + 1):
                robot_locations[i] = robot_positions[i]
                rospy.loginfo(f"Robot {i} initialized at {robot_locations[i]}")
            break
        rate.sleep()

    while not rospy.is_shutdown():
        for i in range(1, NUM_ROBOTS + 1):
            if robot_states[i] == RobotState.QUEUING:
                try:
                    response = signal_queue(botID=i)
                    if response.success:
                        robot_states[i] = RobotState.WAITING_FOR_ORDER
                except rospy.ServiceException:
                    pass

            # Replace the relevant part in controller_main()
            elif robot_states[i] == RobotState.WAITING_FOR_ORDER:
                try:
                    response = listen_order(botId=i)
                    if response.orderTaken:
                        rospy.loginfo(f"ROBOT CONTROLLER::Robot {i} received order to deliver to ({response.deliveryLocationX}, {response.deliveryLocationY})")
                        #robot_paths[i].clear()
                        robot_deliveries[i] = (response.deliveryLocationX, response.deliveryLocationY)

                        delivery_x, delivery_y = robot_deliveries[i]

                        # Update PAT world.csp with current and delivery positions
                        robot_helpers[i].update_world_csp(
                            current_x=robot_locations[i][0],
                            current_y=robot_locations[i][1],
                            delivery_x=delivery_x,
                            delivery_y=delivery_y
                        )

                        # Add a short delay to ensure file is fully written
                        time.sleep(0.5)

                        # Get path to delivery location with retry
                        path = robot_helpers[i].get_path("delivering", delivery_x=delivery_x, delivery_y=delivery_y)
                        if not path:
                            rospy.logwarn(f"ROBOT CONTROLLER::Robot {i} got empty path. Retrying after short delay...")
                            time.sleep(0.5)
                            path = robot_helpers[i].get_path("delivering", delivery_x=delivery_x, delivery_y=delivery_y)

                        robot_paths[i] = deque(path)
                        robot_states[i] = RobotState.DELIVERING
                        rospy.loginfo(f"Robot {i} path: {list(robot_paths[i])}")
                except rospy.ServiceException:
                    pass

            elif robot_states[i] == RobotState.DELIVERING:
                move_robot(i, update_pos, update_grid, set_model_state)
                if not robot_paths[i]:
                    # Check if the robot has reached the delivery location
                    delivery_x, delivery_y = robot_deliveries[i]
                    current_x, current_y = robot_locations[i]
                    if (current_x, current_y) == (delivery_x, delivery_y):
                        rospy.loginfo(f"Robot {i} successfully delivered the order to ({delivery_x}, {delivery_y})")
                        robot_states[i] = RobotState.GOING_HOME
                        robot_paths[i].clear()  # Clear previous path
                        robot_paths[i] = deque(robot_helpers[i].get_path("home"))
                    else:
                        rospy.logwarn(f"Robot {i} failed to deliver the order to ({delivery_x}, {delivery_y})")

            elif robot_states[i] == RobotState.GOING_HOME:
                move_robot(i, update_pos, update_grid, set_model_state)
                if not robot_paths[i]:
                    robot_states[i] = RobotState.QUEUING

        rate.sleep()

if __name__ == '__main__':
    try:
        controller_main()
    except rospy.ROSInterruptException:
        pass
