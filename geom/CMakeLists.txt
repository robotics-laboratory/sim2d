cmake_minimum_required(VERSION 3.5)
project(geom)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/angle.cpp
    src/angle_vector.cpp
    #src/arc.cpp
    src/bezier.cpp
    src/bounding_box.cpp
    src/circle.cpp
    src/complex_polygon.cpp
    src/distance.cpp
    src/intersection.cpp
    src/line.cpp
    src/motion_state.cpp
    src/msg.cpp
    src/polygon.cpp
    #src/polyline.cpp
    src/pose.cpp
    src/segment.cpp
    src/transform.cpp
    src/vector.cpp
    src/vector3.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    common
    geometry_msgs
    nav_msgs
    tf2
    Boost
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common geometry_msgs nav_msgs tf2 Boost)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()