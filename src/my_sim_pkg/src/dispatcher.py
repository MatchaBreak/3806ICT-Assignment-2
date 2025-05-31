
import rospy
import random
from copy import deepcopy
from networkx import DiGraph, from_numpy_array, relabel_nodes, set_node_attributes
from numpy import array
from vrpy import VehicleRoutingProblem
import time
import os

from my_sim_pkg.srv import (
    GetHouseLocations,
    ListenToQueue,
    SignalOrderReady,
    ListenForOrderTaken,
    SignalOrderReadyRequest
)

from settings.load_yaml import load_shared_settings


class Dispatcher:
    def __init__(self):
        rospy.init_node("dispatcher")

        # Load shared settings using the helper function
        self.settings = load_shared_settings()
        if not self.settings:
            rospy.logerr("DISPATCHER::Failed to load shared settings. Using default values.")
            self.settings = {}

        # Initialize dispatcher parameters from YAML
        self.seed = self.settings.get("SEED", 1)
        self.min_pizzas = self.settings.get("MIN_PIZZAS", 1)
        self.max_pizzas = self.settings.get("MAX_PIZZAS", 5)
        self.load_capacity = self.settings.get("LOAD_CAPACITY", 10)
        self.stops_per_trip = self.settings.get("STOPS_PER_TRIP", 4)
        self.total_pizzas_goal = self.settings.get("TOTAL_PIZZAS_GOAL", 20)
        self.pizzas_delivered = 0
        self.start_time = time.time()

        # For the output into OUTPUT_TESTS
        self.output_dir = os.path.join(os.path.dirname(__file__), "OUTPUT_TESTS")
        os.makedirs(self.output_dir, exist_ok=True)

        rospy.loginfo(f"DISPATCHER::Dispatcher initialized with settings: {self.settings}")

        # Wait for services
        rospy.wait_for_service("/get_house_locations")
        rospy.wait_for_service("/listen_to_queue")
        rospy.wait_for_service("/signal_order_ready")
        rospy.wait_for_service("/listen_for_order_taken")

        self.get_houses = rospy.ServiceProxy("/get_house_locations", GetHouseLocations)
        self.listen_queue = rospy.ServiceProxy("/listen_to_queue", ListenToQueue)
        self.signal_ready = rospy.ServiceProxy("/signal_order_ready", SignalOrderReady)
        self.listen_taken = rospy.ServiceProxy("/listen_for_order_taken", ListenForOrderTaken)

        self.rate = rospy.Rate(1)
        self.route_queue = []

        self.houses = []
        self.building_positions = []
        self.demand_each_house = {}

        # Track return times for robots
        self.robot_return_times = {}

        self.setup()

    def create_distance_matrix(self, locations):
        def _manhattan_distance(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        if not locations:
            return []
        depot_loc = locations[0]
        customer_locs = locations[1:]
        n_customers = len(customer_locs)
        matrix_size = n_customers + 2
        dist_matrix = [[0] * matrix_size for _ in range(matrix_size)]
        for j in range(n_customers):
            customer_matrix_idx = j + 1
            dist_matrix[0][customer_matrix_idx] = _manhattan_distance(depot_loc, customer_locs[j])
        for i in range(n_customers):
            matrix_row_idx = i + 1
            for j in range(n_customers):
                matrix_col_idx = j + 1
                dist_matrix[matrix_row_idx][matrix_col_idx] = _manhattan_distance(customer_locs[i], customer_locs[j])
            dist_matrix[matrix_row_idx][-1] = _manhattan_distance(customer_locs[i], depot_loc)
        return dist_matrix

    def setup(self):
        try:
            response = self.get_houses()
            data = response.locations.data
            self.houses = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]
        except rospy.ServiceException:
            rospy.logerr("DISPATCHER::failed to get house locations")
            return

        rospy.loginfo(f"DISPATCHER::initialised with {len(self.houses)} house(s)")
        for idx, house in enumerate(self.houses, 1):
            rospy.loginfo(f"DISPATCHER::House {idx} is located at {house}")

        self.demand_each_house = {i: random.randint(self.min_pizzas, self.max_pizzas) for i in range(len(self.houses))}
        self.building_positions = [(8, 8)] + [deepcopy(h) for h in self.houses]
        self.distance_matrix = self.create_distance_matrix(self.building_positions)

    def generate_routes(self):
        # Filter out houses with zero demand
        active_houses = {i: demand for i, demand in self.demand_each_house.items() if demand > 0}

        # If no active houses remain, stop generating routes
        if not active_houses:
            rospy.loginfo("DISPATCHER::All houses have received their pizzas. No routes to generate.")
            return

        # Create distance matrix for active houses
        A = array(self.distance_matrix, dtype=[("cost", int)])
        G = from_numpy_array(A, create_using=DiGraph())
        set_node_attributes(G, values=active_houses, name="demand")
        G = relabel_nodes(G, {0: "Source", len(self.building_positions): "Sink"})
        prob = VehicleRoutingProblem(G, load_capacity=self.load_capacity)
        prob.num_stops = self.stops_per_trip
        prob.solve()

        routes = prob.best_routes
        sorted_routes = []
        while routes:
            _, route = routes.popitem()
            sorted_routes.append(route)
        sorted_routes.reverse()

        for route in sorted_routes:
            for i in range(len(route)):
                if route[i] == "Source":
                    route[i] = -100
                elif route[i] == "Sink":
                    route[i] = -200

        for route in sorted_routes:
            coords = []
            for node in route:
                if node not in (-100, -200):
                    x, y = self.building_positions[node]
                    coords.extend([x, y])
            self.route_queue.append(coords)

    def dispatch_loop(self):
        while not rospy.is_shutdown() and self.pizzas_delivered < self.total_pizzas_goal:
            try:
                queue_response = self.listen_queue()
                bot_id = queue_response.nextInQueue
                if bot_id < 0:
                    rospy.loginfo("DISPATCHER::No robots in queue.")
                    rospy.sleep(2)
                    continue
            except rospy.ServiceException:
                rospy.logerr("DISPATCHER::Failed to listen to queue")
                self.rate.sleep()
                continue

            rospy.loginfo(f"DISPATCHER::Robot {bot_id} is next in queue")
            if not self.route_queue:
                self.generate_routes()
            if not self.route_queue:
                continue

            delivery = self.route_queue.pop(0)
            order_ready = SignalOrderReadyRequest()
            order_ready.botID = bot_id
            order_ready.deliveryLocations = delivery

            while not rospy.is_shutdown():
                try:
                    response = self.signal_ready(order_ready)
                    if response.success:
                        rospy.loginfo(f"DISPATCHER::Robot {bot_id} notified successfully")
                        break
                    else:
                        rospy.loginfo("DISPATCHER::Waiting for previous robot to pick up order")
                except rospy.ServiceException:
                    rospy.logerr("DISPATCHER::Failed to signal order readiness")
                rospy.sleep(2)

            try:
                self.listen_taken()
                rospy.loginfo(f"DISPATCHER::Robot {bot_id} has taken the order")
            except rospy.ServiceException:
                rospy.logwarn("DISPATCHER::Failed to confirm order taken")

            # Record the return time for the robot
            self.robot_return_times[bot_id] = time.time()

            self.pizzas_delivered += len(delivery) // 2
            self.rate.sleep()

        # Write summary after all pizzas are delivered
        self.write_summary()
            
    def write_summary(self):
        end_time = time.time()
        total_time = end_time - self.start_time
        avg_time = total_time / self.pizzas_delivered if self.pizzas_delivered > 0 else 0

        summary = (
            f"SEED: {self.seed}\n"
            f"NUM_OF_HOUSES: {len(self.houses)}\n"
            f"NUM_OF_OBSTACLES: {self.settings.get('NUM_OBSTACLES', 'N/A')}\n"
            f"STOPS_PER_TRIP: {self.stops_per_trip}\n"
            f"NUM_OF_PIZZAS_DELIVERED: {self.pizzas_delivered}\n"
            f"TOTAL_TIME: {total_time:.2f} seconds\n"
            f"AVG_TIME_PER_PIZZA: {avg_time:.2f} seconds\n"
            f"DISPATCHER PARAMETERS:\n"
            f"  MIN_PIZZAS: {self.min_pizzas}\n"
            f"  MAX_PIZZAS: {self.max_pizzas}\n"
            f"  LOAD_CAPACITY: {self.load_capacity}\n"
            f"  TOTAL_PIZZAS_GOAL: {self.total_pizzas_goal}\n"
        )

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"{timestamp}_test.txt"
        filepath = os.path.join(self.output_dir, filename)

        with open(filepath, "w") as f:
            f.write(summary)

        rospy.loginfo(f"DISPATCHER::Summary written to {filepath}")
        

if __name__ == "__main__":
    try:
        d = Dispatcher()
        d.dispatch_loop()
        d.write_summary()
    except rospy.ROSInterruptException:
        pass