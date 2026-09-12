cmake_minimum_required(VERSION 3.8)
project(amr_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

install(
  DIRECTORY
    launch
    models
    worlds
    maps
    config
    dashboard
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_map.py
    scripts/navigate_choke_point.py
    scripts/benchmark_logger.py
    scripts/uncoordinated_choke_test.py
    scripts/intent_broadcaster.py
    scripts/decentralized_coordinator.py
    scripts/coordinated_choke_test.py
    scripts/battery_simulator.py
    scripts/task_allocator_cnp.py
    scripts/fleet_dashboard_server.py
    scripts/benchmark_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
