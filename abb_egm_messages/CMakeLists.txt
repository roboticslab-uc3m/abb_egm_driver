cmake_minimum_required(VERSION 3.5)

project(abb_egm_messages)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuidanceMode.msg"
  "srv/SwitchMode.srv"
)
