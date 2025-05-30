cmake_minimum_required(VERSION 3.0.2)
project(my_sim_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  message_generation
)


add_service_files(
  FILES
    grid/GetHouseLocations.srv
    grid/UpdateCurrentBotPosition.srv
    grid/UpdateGrid.srv
    relay/ListenForOrderTaken.srv
    relay/ListenToOrderStatus.srv
    relay/ListenToQueue.srv
    relay/SignalOrderReady.srv
    relay/SignalQueuedUp.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs gazebo_msgs
)


catkin_install_python(PROGRAMS
  src/grid_manager.py
  src/relay_server.py
  src/dispatcher.py
  src/robot_controller_node.py
  scripts/spawn_houses.py 
  scripts/spawn_obstacles.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)




