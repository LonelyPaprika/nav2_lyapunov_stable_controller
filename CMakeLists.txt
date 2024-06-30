cmake_minimum_required(VERSION 3.8)
project(nav2_lyapunov_stable_controller)

# find dependencies
set(DEPENDENCIES
  ament_cmake
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_2d_utils
  tf2
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  pluginlib
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

nav2_package()

include_directories(
  include
)

add_library(nav2_lyapunov_stable_controller SHARED
        src/lyapunov_stable_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_lyapunov_stable_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_lyapunov_stable_controller
  ${DEPENDENCIES}
)

install(TARGETS nav2_lyapunov_stable_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
  MAX_LINE_LENGTH 110
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_lyapunov_stable_controller)
ament_export_dependencies(${DEPENDENCIES})

pluginlib_export_plugin_description_file(nav2_core nav2_lyapunov_stable_controller.xml)

ament_package()
