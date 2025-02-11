cmake_minimum_required(VERSION 3.5)
project(kondo_ics_servo_driver)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# サービスの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPosition.srv"
  "srv/GetPosition.srv"
  "srv/SetID.srv"
  "srv/GetID.srv"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

# ノードのコンパイル対象にics_driver.cppを追加
add_executable(kondo_ics_servo_driver_node
  src/kondo_ics_servo_driver_node.cpp
  src/ics_driver.cpp
)

ament_target_dependencies(kondo_ics_servo_driver_node rclcpp)

# サービスインターフェースの依存関係をリンク
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kondo_ics_servo_driver_node ${cpp_typesupport_target})

install(TARGETS
  kondo_ics_servo_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()