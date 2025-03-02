cmake_minimum_required(VERSION 3.5)
project(icuas25_sm)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY icuas25_sm
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    icuas25_sm/MainStateMachine.py
    icuas25_sm/Data_Wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
