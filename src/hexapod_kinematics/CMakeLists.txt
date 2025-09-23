cmake_minimum_required(VERSION 3.8)
project(hexapod_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegJoints.msg"
  "msg/MultiLegJoints.msg"
  "msg/LegTarget.msg"
  "msg/MultiLegTargets.msg"
  DEPENDENCIES geometry_msgs
)

install(
    PROGRAMS
    hexapod_kinematics/kinematics_engine.py
    hexapod_kinematics/interactive_ik_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
