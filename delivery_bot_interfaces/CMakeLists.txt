cmake_minimum_required(VERSION 3.8)
project(delivery_bot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generiere Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeliveryTask.msg"
  "msg/RobotStatus.msg"
  "msg/BatteryAlert.msg"
  "srv/DeliverPackage.srv"
  "srv/ChargeBattery.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()