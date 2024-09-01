# cmake_minimum_required(VERSION 3.5)
# project(my_actuator_ros2)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# # Find dependencies
# find_package(ament_cmake REQUIRED) 
# find_package(rclcpp REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(interfaces REQUIRED)
# find_package(CppLinuxSerial REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)


# set(dependencies
#    interfaces
#    ament_cmake
# )

# # Include directories
# include_directories(
#   include
#   launch
#   ${CMAKE_CURRENT_SOURCE_DIR}/install/interfaces/include
#   ${CMAKE_CURRENT_SOURCE_DIR}RmdApiSerial/rmd-sdk/include
#   )
  
# add_subdirectory(RmdApiSerial/rmd-sdk)
# ament_export_dependencies(rosidl_default_runtime)   
# add_executable(my_actuator_control src/my_actuator_control.cpp )
# add_executable(joy_teleop src/joy_teleop.cpp)
# ament_target_dependencies(joy_teleop ${CMAKE_LIBRARIES} rclcpp std_msgs std_srvs geometry_msgs interfaces sensor_msgs)
# target_link_libraries(joy_teleop ${CMAKE_LIBRARIES} CppLinuxSerial Rmd_Library)
# target_include_directories(joy_teleop PUBLIC ${CMAKE_INCLUDE_DIRS})
# ament_target_dependencies(my_actuator_control ${CMAKE_LIBRARIES} rclcpp std_msgs std_srvs geometry_msgs interfaces sensor_msgs)
# target_link_libraries(my_actuator_control ${CMAKE_LIBRARIES} CppLinuxSerial Rmd_Library)
# install(TARGETS
# # testka
# # toggle_joy
# joy_teleop
# my_actuator_control
# DESTINATION lib/${PROJECT_NAME}
# )
# install(
#   DIRECTORY include launch
#   DESTINATION share/${PROJECT_NAME}

# )
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
# #ament_package()
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(${dependencies})
# ament_package()

cmake_minimum_required(VERSION 3.5)  # Specify the minimum CMake version required
project(my_actuator_ros2)            # Define the project name

# Default to C99 if not set
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set compiler-specific flags for GNU and Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages and dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(CppLinuxSerial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# List of dependencies to export
set(dependencies
   interfaces
   ament_cmake
)

# Include directories for the project
include_directories(
  include
  launch
  ${CMAKE_CURRENT_SOURCE_DIR}/install/interfaces/include
  ${CMAKE_CURRENT_SOURCE_DIR}/RmdApiSerial/rmd-sdk/include
)

# Add subdirectory for RmdApiSerial library
add_subdirectory(RmdApiSerial/rmd-sdk)

# Export dependencies for runtime
ament_export_dependencies(rosidl_default_runtime)

# Define executables
add_executable(my_actuator_control src/my_actuator_control.cpp)
add_executable(joy_teleop src/joy_teleop.cpp)

# Link libraries and dependencies to the executables
ament_target_dependencies(joy_teleop ${CMAKE_LIBRARIES} rclcpp std_msgs std_srvs geometry_msgs interfaces sensor_msgs)
target_link_libraries(joy_teleop ${CMAKE_LIBRARIES} CppLinuxSerial Rmd_Library)
target_include_directories(joy_teleop PUBLIC ${CMAKE_INCLUDE_DIRS})

ament_target_dependencies(my_actuator_control ${CMAKE_LIBRARIES} rclcpp std_msgs std_srvs geometry_msgs interfaces sensor_msgs)
target_link_libraries(my_actuator_control ${CMAKE_LIBRARIES} CppLinuxSerial Rmd_Library)

# Install targets
install(TARGETS
  joy_teleop
  my_actuator_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

# Add testing support if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following lines if necessary
  # set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright linter
  # set(ament_cmake_cpplint_FOUND TRUE)    # Skip cpplint if not in a git repo
  ament_lint_auto_find_test_dependencies()
endif()

# Export include directories and libraries
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

# Finalize the package
ament_package()
