cmake_minimum_required(VERSION 3.8)
project(cod_serial)

find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)

add_executable(cod_serial  src/uart_transporter.cpp
        src/cod_serial.cpp
)
ament_target_dependencies(cod_serial
        rclcpp
        rm_interfaces
)
target_link_libraries(cod_serial
    Eigen3::Eigen
)

install(TARGETS cod_serial
        DESTINATION lib/${PROJECT_NAME}
)
ament_package()
