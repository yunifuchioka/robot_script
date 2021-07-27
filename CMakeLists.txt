cmake_minimum_required(VERSION 3.5)
project(robot_script)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(real_time_tools REQUIRED)

include_directories(
  include/robot_script
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(main src/main.cpp)

# ament_target_dependencies(main Eigen3 real_time_tools)
target_link_libraries(main Eigen3::Eigen real_time_tools::real_time_tools)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
