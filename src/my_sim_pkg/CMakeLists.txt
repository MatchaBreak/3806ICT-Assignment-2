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
    grid/FileLockRequest.srv
    grid/FileUnlockRequest.srv
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
  scripts/grid_manager.py
  scripts/relay_server.py
  scripts/dispatcher.py
  scripts/robot_controller_node.py
  scripts/spawn_houses.py 
  scripts/spawn_obstacles.py 
  #python_pkgs/pat_helper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY python_pkgs/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)




