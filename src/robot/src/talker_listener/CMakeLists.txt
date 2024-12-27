cmake_minimum_required(VERSION 3.5)
project(talker_listener)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Inclui diretórios de cabeçalho
include_directories(include)

# Compila os executáveis para Talker e Listener
add_executable(talker_node src/talker_main.cpp src/talker.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)

add_executable(listener_node src/listener_main.cpp src/listener.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)

# Instala os executáveis
install(TARGETS
  talker_node
  listener_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
