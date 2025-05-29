#!/usr/bin/env python3
import rospy
import random
from my_sim_pkg.srv import (
    GetHouseLocations,
    ListenToQueue,
    SignalOrderReady,
    ListenForOrderTaken,
    SignalOrderReadyRequest
)

def dispatcher():
    rospy.init_node("dispatcher")

    # Wait for services to be available
    rospy.wait_for_service("/get_house_locations")
    rospy.wait_for_service("/listen_to_queue")
    rospy.wait_for_service("/signal_order_ready")
    rospy.wait_for_service("/listen_for_order_taken")

    get_houses = rospy.ServiceProxy("/get_house_locations", GetHouseLocations)
    listen_queue = rospy.ServiceProxy("/listen_to_queue", ListenToQueue)
    signal_ready = rospy.ServiceProxy("/signal_order_ready", SignalOrderReady)
    listen_taken = rospy.ServiceProxy("/listen_for_order_taken", ListenForOrderTaken)

    # Step 1: Get all house positions
    rospy.loginfo("dispatcher: requesting house locations")
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

    rate = rospy.Rate(1)  # Slow down loop for readability

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

        # Choose a random house
        house = random.choice(houses)
        rospy.loginfo("dispatcher: assigning delivery to house at (%d, %d)", *house)

        # Signal order readiness
        order_ready = SignalOrderReadyRequest()
        order_ready.botID = bot_id
        order_ready.deliveryLocationX = house[0]
        order_ready.deliveryLocationY = house[1]

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

        # Wait for order to be picked up
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


