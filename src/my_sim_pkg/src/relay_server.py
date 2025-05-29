#!/usr/bin/env python3
import rospy
from my_sim_pkg.srv import (
    SignalQueuedUp, SignalQueuedUpResponse,
    ListenToQueue, ListenToQueueResponse,
    SignalOrderReady, SignalOrderReadyResponse,
    ListenToOrderStatus, ListenToOrderStatusResponse,
    ListenForOrderTaken, ListenForOrderTakenResponse,
)
from collections import deque

# Robot queue
robot_queue = deque()

# Delivery signals
signal_order_ready = -1
delivery_location = (0, 0)
order_taken = False
available_orders = deque()

def handle_signal_queued_up(req):
    if req.botID in robot_queue:
        rospy.loginfo(f"RELAY_SERVER::Robot {req.botID} failed to queue up!")
        return SignalQueuedUpResponse(success=False)
    robot_queue.append(req.botID)
    rospy.loginfo(f"RELAY_SERVER::Robot {req.botID} has queued up! Queue: {list(robot_queue)}")
    return SignalQueuedUpResponse(success=True)


def handle_listen_to_queue(req):
    if robot_queue:
        next_bot = robot_queue.popleft()
        rospy.loginfo(f"RELAY_SERVER::Robot {next_bot} dequeued for dispatch.")
        return ListenToQueueResponse(nextInQueue=next_bot)
    rospy.loginfo("RELAY_SERVER::Queue is empty.")
    return ListenToQueueResponse(nextInQueue=-1)


def handle_signal_order_ready(req):
    global signal_order_ready, delivery_location
    if signal_order_ready >= 0:
        return SignalOrderReadyResponse(success=False)
    signal_order_ready = req.botID
    delivery_location = (req.deliveryLocationX, req.deliveryLocationY)
    rospy.loginfo(f"RELAY_SERVER::Order ready for robot {req.botID} at {delivery_location}")
    return SignalOrderReadyResponse(success=True)


def handle_listen_to_order_status(req):
    global order_taken
    if signal_order_ready == req.botId:
        order_taken = True
        rospy.loginfo(f"RELAY_SERVER::Robot {req.botId} took the order to {delivery_location}")
        return ListenToOrderStatusResponse(
            orderTaken=True,
            deliveryLocationX=delivery_location[0],
            deliveryLocationY=delivery_location[1]
        )
    rospy.loginfo(f"RELAY_SERVER::Robot {req.botId} failed to take order {signal_order_ready}")
    return ListenToOrderStatusResponse(orderTaken=False)


def handle_listen_for_order_taken(req):
    global signal_order_ready, order_taken
    if order_taken:
        rospy.loginfo("Order taken. Resetting state.")
        signal_order_ready = -1
        order_taken = False
        return ListenForOrderTakenResponse(taken=True)
    rospy.loginfo("No order has been taken yet.")
    return ListenForOrderTakenResponse(taken=False)


def relay_server():
    rospy.init_node("relay_server")

    rospy.Service("/signal_queued_up", SignalQueuedUp, handle_signal_queued_up)
    rospy.Service("/listen_to_queue", ListenToQueue, handle_listen_to_queue)
    rospy.Service("/signal_order_ready", SignalOrderReady, handle_signal_order_ready)
    rospy.Service("/listen_to_order_status", ListenToOrderStatus, handle_listen_to_order_status)
    rospy.Service("/listen_for_order_taken", ListenForOrderTaken, handle_listen_for_order_taken)

    rospy.loginfo("Relay server is running.")
    rospy.spin()


if __name__ == '__main__':
    try:
        relay_server()
    except rospy.ROSInterruptException:
        pass
