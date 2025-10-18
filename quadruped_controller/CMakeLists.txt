cmake_minimum_required(VERSION 3.5)
project(quadruped_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  
)
install(PROGRAMS
  src/robot_controller_gazebo.py
  src/QuadrupedOdometryNode.py
  src/torso_circle.py
  src/torso_dance.py
  src/torso_wave.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/cmd_vel_pub.py
  src/robot_mode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()