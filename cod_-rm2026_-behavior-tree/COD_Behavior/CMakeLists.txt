cmake_minimum_required(VERSION 3.8...3.31)
project(cod_behavior)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

add_executable(tree_1 src/cod_behavior.cpp)
add_executable(test_1 test/test_GoZone.cpp)
add_executable(test_2 test/test_OnZone.cpp)
add_executable(test_3 test/test_LowHp.cpp)
add_executable(test_4 test/test_Recover.cpp)

ament_target_dependencies(tree_1
        rclcpp
        nav2_msgs
        example_interfaces
        rclcpp_action
        geometry_msgs
        behaviortree_cpp
        rm_interfaces
        tf2_geometry_msgs
        tf2_ros
        behaviortree_ros2
)

ament_target_dependencies(test_1
        rclcpp
        rm_interfaces
)
ament_target_dependencies(test_2
        rclcpp
        rm_interfaces
)
ament_target_dependencies(test_3
        rclcpp
        rm_interfaces
)
ament_target_dependencies(test_4
        rclcpp
        rm_interfaces
)

include_directories(
        include
)

install(TARGETS tree_1
        test_1
        test_2
        test_3
        test_4
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
ament_package()