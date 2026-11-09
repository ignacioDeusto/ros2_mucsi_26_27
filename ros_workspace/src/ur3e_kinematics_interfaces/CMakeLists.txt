cmake_minimum_required(VERSION 3.8)
project(ur3e_kinematics_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Genera el código Python (y C++) del mensaje a partir del archivo .msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstadoCinematico.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
