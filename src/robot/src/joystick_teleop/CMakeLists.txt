cmake_minimum_required(VERSION 3.16.3)
project(joystick_teleop)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências do projeto
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Diretórios de include
include_directories(include)

# Compilação da biblioteca principal
add_library(${PROJECT_NAME}_lib src/joystick_teleop.cpp)

# Dependências da biblioteca
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp sensor_msgs geometry_msgs)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Executável principal
add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# Linkando o executável com a biblioteca
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Instalação dos arquivos
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_lib
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)
ament_package()
