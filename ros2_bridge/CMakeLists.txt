cmake_minimum_required(VERSION 3.8)
project(stm32_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  stm32_bridge/bridge_node.py
  stm32_bridge/ota_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE "execute_process(COMMAND chmod +x \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/bridge_node.py\" \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ota_client.py\")")

install(DIRECTORY
  stm32_bridge/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "bridge_node.py" EXCLUDE
  PATTERN "ota_client.py" EXCLUDE
)

ament_package()
