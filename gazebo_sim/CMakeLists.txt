cmake_minimum_required(VERSION 3.8)
project(stm32_h750_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  gazebo_sim/motor_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
