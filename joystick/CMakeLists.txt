cmake_minimum_required(VERSION 3.5)
project(joystick)

# Найдем необходимые пакеты ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Установим директорию для Python-скриптов
install(PROGRAMS
  scripts/ramped_joystick.py  # Ваш скрипт
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch файлов
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Добавляем зависимости для сборки
ament_package()