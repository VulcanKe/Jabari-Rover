cmake_minimum_required(VERSION 3.8)
project(manual_control)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/manual_control_node.py
  scripts/test_manual_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install other resources
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(python3-pytest REQUIRED)
  
  # Run linting tests
  ament_lint_auto_find_test_dependencies()
  
  # Add Python tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(manual_control_tests test/
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endif()

# Export package information
ament_package()