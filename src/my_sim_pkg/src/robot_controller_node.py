# robot_controller_node.py
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState #for obstacle knowledge omniscience
from std_msgs.msg import Int32MultiArray
from collections import deque
from threading import Lock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelStateRequest
from my_sim_pkg.srv import UpdateGrid #to update grid manager's world state (authority of world state)
import threading
from my_sim_pkg.srv import (
    ListenToOrderStatus,
    SignalQueuedUp,
    UpdateCurrentBotPosition,
)

from world import World, TileType
from settings.load_yaml import load_shared_settings

from robot import Robot

# Executor for managing multiple robots in a simulation environment
class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller_node')
        
        self.settings = load_shared_settings()
        self.NUM_ROBOTS = self.settings.get("NUM_ROBOTS", 4)
        self.TILE_SIZE = self.settings.get("TILE_SIZE", 1)
        self.NUM_OBSTACLES = self.settings.get("NUM_OBSTACLES", 4)
       
        self.latest_world = World()
        self.lock = Lock()

        self.robot_positions = {}
        self.robot_locations = {}
        
        #list of all unknown obstacles. Each time a robot detects an obstacle, it removes it from this list
        self.unknown_obstacles = []
        #after discovering an obstacle, add to this list so bots already on a path with 
        #the detected obstacle in it can still avoid it (before the local modal is updated 
        # for future pathfinding)
        self.known_obstacles = []

        self.directions = {
            (0, -1): "moveLeft",
            (0, 1): "moveRight",
            (-1, 0): "moveUp",
            (1, 0): "moveDown",
        }

        self.signal_queue = rospy.ServiceProxy('/signal_queued_up', SignalQueuedUp)
        self.listen_order = rospy.ServiceProxy('/listen_to_order_status', ListenToOrderStatus)
        self.update_pos = rospy.ServiceProxy('/update_current_bot_position', UpdateCurrentBotPosition)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.update_grid_srv = rospy.ServiceProxy('/update_grid', UpdateGrid)
        rospy.Subscriber('/world_grid', Int32MultiArray, self.world_callback)

        self.robots = {
            i: Robot(
                robot_id=i,
                controller=self,
                signal_queue=self.signal_queue,
                listen_order=self.listen_order
            ) for i in range(1, self.NUM_ROBOTS + 1)
        }

        for i in range(1, self.NUM_ROBOTS + 1):
            rospy.Subscriber(f"/turtlebot3_burger_{i}/odom", Odometry, self.odom_callback, i)

        self.wait_for_odometry()

        # Assign initial positions to robot instances
        for i, robot in self.robots.items():
            robot.current_location = self.robot_locations[i]

        # Load known unknowns for robots and the dispatcher to discover
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        for i in range(1, self.NUM_OBSTACLES + 1):
            name = f"cardboard_box_{i}"
            try:
                res = get_model(name, "world")
                x = int(round(res.pose.position.x / self.TILE_SIZE))
                y = int(round(res.pose.position.y / self.TILE_SIZE))
                self.unknown_obstacles.append((x, y))
                rospy.loginfo(f"ROBOT CONTROLLER::Added obstacle candidate at ({x}, {y})")
            except rospy.ServiceException as e:
                rospy.logwarn(f"ROBOT CONTROLLER::Failed to locate {name}: {e}")

        self.controller_loop()

    def mark_obstacle(self, x, y):
        try:
            res = self.update_grid_srv(tilePositionX=x, tilePositionY=y, tileType=TileType.OBSTACLE)
            if res.success:
                rospy.loginfo(f"Marked obstacle at ({x}, {y}) via UpdateGrid")
            else:
                rospy.logwarn(f"Failed to mark obstacle at ({x}, {y}) via UpdateGrid")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to UpdateGrid failed: {e}")
    
    def odom_callback(self, msg, bot_id):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            grid_x = int(round(x / self.TILE_SIZE))
            grid_y = int(round(y / self.TILE_SIZE))
            self.robot_positions[bot_id] = (grid_x, grid_y)

    #update the controller's understanding of the world grid based on the grid manager's World object
    def world_callback(self, msg):
        data = msg.data
        width = self.latest_world.width
        height = self.latest_world.height
        for i in range(height):
            for j in range(width):
                idx = i * width + j
                self.latest_world.grid[i][j] = data[idx]

    # Wait for all robots to publish their odometry data 
    def wait_for_odometry(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if len(self.robot_positions) == self.NUM_ROBOTS:
                for i in range(1, self.NUM_ROBOTS + 1):
                    self.robot_locations[i] = self.robot_positions[i]
                break
            rate.sleep()

        # Ensure robot_locations are initialized
        for i in range(1, self.NUM_ROBOTS + 1):
            if i not in self.robot_locations:
                self.robot_locations[i] = Settings.robot_positions[i]


    # Teleport a robot to a new position in Gazebo (so the world environment and Gazebo are in sync)
    def teleport_robot(self, bot_id, x, y, steps=10, delay=0.05):
        try:
            # Get the current position of the robot
            current_x, current_y = self.robot_locations.get(bot_id, (0, 0))

            # Calculate intermediate positions
            for step in range(1, steps + 1):
                interpolated_x = current_x + (x - current_x) * step / steps
                interpolated_y = current_y + (y - current_y) * step / steps

                # Set the robot's state for Gazebo
                state = ModelState()
                state.model_name = f"turtlebot3_burger_{bot_id}"
                state.pose.position.x = interpolated_x * self.TILE_SIZE
                state.pose.position.y = interpolated_y * self.TILE_SIZE
                state.pose.position.z = 0.1
                state.pose.orientation.w = 1.0

                req = SetModelStateRequest()
                req.model_state = state

                # Call the Gazebo service to update the robot's position
                self.set_model_state(req)
                rospy.sleep(delay)

            # Update the robot's final position
            self.robot_locations[bot_id] = (x, y)
            #rospy.loginfo(f"ROBOT CONTROLLER::Robot {bot_id} teleported to ({x}, {y})")

        except Exception as e:
            rospy.logwarn(f"ROBOT CONTROLLER::Failed to teleport robot {bot_id}: {e}")
        
    def controller_loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            threads = []

            for robot in self.robots.values():
                thread = threading.Thread(target=robot.update_state)
                thread.start()
                threads.append(thread)

            for thread in threads:
                thread.join()

            rate.sleep()

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
