cmake_minimum_required(VERSION 3.8)
project(tb3_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install directories
install(DIRECTORY
bt_xml launch
DESTINATION share/${PROJECT_NAME}
)

# Install C++ behaviors
set(BEHAVIOR_SOURCES
src/navigation_behaviors.cpp 
)

set(TARGET_DEPENDS
    rclcpp
    behaviortree_cpp_v3
    yaml-cpp
    rclcpp_action
    nav2_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(include)
add_executable(autonomy_node src/autonomy_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(autonomy_node ${TARGET_DEPENDS})
target_link_libraries(autonomy_node ${YAML_CPP_LIBRARIES})

install(TARGETS
autonomy_node
DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
