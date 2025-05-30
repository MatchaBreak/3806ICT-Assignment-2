import rospy
import random
from copy import deepcopy
from networkx import DiGraph, from_numpy_array, relabel_nodes, set_node_attributes
from numpy import array
from vrpy import VehicleRoutingProblem

from my_sim_pkg.srv import (
    GetHouseLocations,
    ListenToQueue,
    SignalOrderReady,
    ListenForOrderTaken,
    SignalOrderReadyRequest
)

min_pizzas_per_order = 1
max_pizzas_per_order = 10
robot_load_capacity = 15
number_of_stops_per_trip = 4

def create_distance_matrix(locations):
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
        current_customer_loc = customer_locs[i]
        matrix_row_idx = i + 1
        for j in range(n_customers):
            matrix_col_idx = j + 1
            dist_matrix[matrix_row_idx][matrix_col_idx] = _manhattan_distance(current_customer_loc, customer_locs[j])
        sink_matrix_idx = matrix_size - 1
        dist_matrix[matrix_row_idx][sink_matrix_idx] = _manhattan_distance(current_customer_loc, depot_loc)
    return dist_matrix

def dispatcher():
    rospy.init_node("dispatcher")
    rospy.wait_for_service("/get_house_locations")
    rospy.wait_for_service("/listen_to_queue")
    rospy.wait_for_service("/signal_order_ready")
    rospy.wait_for_service("/listen_for_order_taken")
    get_houses = rospy.ServiceProxy("/get_house_locations", GetHouseLocations)
    listen_queue = rospy.ServiceProxy("/listen_to_queue", ListenToQueue)
    signal_ready = rospy.ServiceProxy("/signal_order_ready", SignalOrderReady)
    listen_taken = rospy.ServiceProxy("/listen_for_order_taken", ListenForOrderTaken)

    try:
        house_response = get_houses()
    except rospy.ServiceException:
        rospy.logerr("dispatcher: failed to call /get_house_locations")
        return

    houses = []
    data = house_response.locations.data
    for i in range(0, len(data) - 1, 2):
        houses.append((data[i], data[i + 1]))

    if not houses:
        rospy.logwarn("dispatcher: no house locations found")
        return

    rospy.loginfo("dispatcher: initialised with %d house(s)", len(houses))
    for idx, house in enumerate(houses, 1):
        rospy.loginfo("dispatcher: House %d is located at (%d, %d)", idx, house[0], house[1])

    rate = rospy.Rate(1)
    demand_each_house = {i: random.randrange(min_pizzas_per_order, max_pizzas_per_order) for i in range(len(houses))}
    building_positions = [(8, 8)] + [deepcopy(h) for h in houses]
    distance_matrix = create_distance_matrix(building_positions)

    while not rospy.is_shutdown():
        try:
            queue_response = listen_queue()
        except rospy.ServiceException:
            rospy.logerr("dispatcher: failed to call /listen_to_queue")
            rate.sleep()
            continue

        bot_id = queue_response.nextInQueue
        if bot_id < 0:
            rospy.loginfo("dispatcher: No robots in queue.")
            rospy.sleep(2)
            continue

        rospy.loginfo("dispatcher: Robot %d is next in queue", bot_id)
        house = random.choice(houses)
        rospy.loginfo("dispatcher: assigning delivery to house at (%d, %d)", *house)

        A = array(distance_matrix, dtype=[("cost", int)])
        G = from_numpy_array(A, create_using=DiGraph())
        set_node_attributes(G, values=demand_each_house, name="demand")
        G = relabel_nodes(G, {0: "Source", len(building_positions): "Sink"})

        prob = VehicleRoutingProblem(G, load_capacity=robot_load_capacity)
        prob.num_stops = number_of_stops_per_trip
        prob.solve()

        best_routes = prob.best_routes
        sorted_house_routes_list = []
        while best_routes:
            _, route = best_routes.popitem()
            sorted_house_routes_list.append(route)
        sorted_house_routes_list.reverse()

        for route in sorted_house_routes_list:
            for i in range(len(route)):
                if route[i] == "Source":
                    route[i] = -100
                elif route[i] == "Sink":
                    route[i] = -200

        converted_routes = []
        for route in sorted_house_routes_list:
            coords = []
            for node in route:
                if node not in (-100, -200):
                    x, y = building_positions[node]
                    coords.extend([x, y])
            converted_routes.append(coords)

        if not converted_routes:
            continue

        order_ready = SignalOrderReadyRequest()
        order_ready.botID = bot_id
        order_ready.deliveryLocations = converted_routes[0]

        while not rospy.is_shutdown():
            try:
                ready_response = signal_ready(order_ready)
                if ready_response.success:
                    rospy.loginfo("dispatcher: robot %d notified successfully", bot_id)
                    break
                else:
                    rospy.loginfo("dispatcher: waiting for previous robot to pick up order")
            except rospy.ServiceException:
                rospy.logerr("dispatcher: failed to call /signal_order_ready")
            rospy.sleep(2)

        try:
            listen_taken()
            rospy.loginfo("dispatcher: robot %d has taken the order", bot_id)
        except rospy.ServiceException:
            rospy.logwarn("dispatcher: failed to confirm order taken")

        rate.sleep()

if __name__ == "__main__":
    try:
        dispatcher()
    except rospy.ROSInterruptException:
        pass
