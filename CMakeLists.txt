cmake_minimum_required(VERSION 3.8)
project(trajectory_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(RL REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(trajectory_node src/trajectory_node.cpp)

ament_target_dependencies(trajectory_node
        trajectory_msgs
        rclcpp
        RL)


install(TARGETS trajectory_node
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        )

install(TARGETS trajectory_node
        RUNTIME DESTINATION lib/trajectory_node
        )

install(DIRECTORY include/
        DESTINATION include
        )

if(BUILD_TESTING)

endif()

# Install Python modules
#ament_python_install_package(trajectory_node SCRIPTS_DESTINATION lib/trajectory_node)

ament_export_libraries(

)
ament_export_include_directories(
        include
)
ament_export_dependencies(
        rclcpp
)

ament_package()
