cmake_minimum_required(VERSION 3.8)
project(single_drone_flight)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# C++ executable
add_executable(single_drone_control_cpp controller/SingleDroneControl.cpp)
ament_target_dependencies(single_drone_control_cpp rclcpp px4_msgs geometry_msgs)

install(TARGETS
        single_drone_control_cpp
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# Install Python executables directly from controller directory
install(PROGRAMS 
        controller/single_drone_control.py
        controller/simulation_launcher.py
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
