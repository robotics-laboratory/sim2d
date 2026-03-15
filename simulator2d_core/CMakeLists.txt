cmake_minimum_required(VERSION 3.5)
project(simulator2d_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(model REQUIRED)
find_package(geom REQUIRED)
find_package(map REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# engine —  физика, без ROS-ноды
add_library(simulator2d_core SHARED
    src/simulator_engine.cpp
    src/truck_state.cpp
    src/simulation_map.cpp
)
ament_target_dependencies(simulator2d_core
    tf2
    tf2_geometry_msgs
    Boost
    Eigen3
    model
    geom
    map
    yaml_cpp_vendor
)
target_include_directories(simulator2d_core
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_export_targets(simulator2d_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(nlohmann_json)
install(
    TARGETS simulator2d_core
    EXPORT simulator2d_core_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# нода — стандартные ROS топики, без truck_msgs
add_executable(${PROJECT_NAME}_node
    src/simulator_node.cpp
    src/main.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    nav_msgs
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    rosgraph_msgs
    sensor_msgs
    nlohmann_json
    model
    geom
)
target_link_libraries(${PROJECT_NAME}_node simulator2d_core)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
        ${PROJECT_NAME}_tests
        test/main.cpp
        test/simulation_map_tests.cpp
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    ament_target_dependencies(${PROJECT_NAME}_tests
        rclcpp
        ament_index_cpp
        map
    )
    target_link_libraries(${PROJECT_NAME}_tests simulator2d_core)
endif()

ament_package()