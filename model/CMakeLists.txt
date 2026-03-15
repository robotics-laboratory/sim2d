cmake_minimum_required(VERSION 3.22)

project(model)

find_package(ament_cmake REQUIRED)
find_package(geom REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(common REQUIRED)
find_package(tf2 REQUIRED)
#find_package(geometry_msgs REQUIRED) 

add_library(model SHARED
    src/params.cpp
    src/shape.cpp
    src/model.cpp
)

ament_target_dependencies(model
    common
    geom
    #geometry_msgs
    tf2
    rclcpp
    yaml_cpp_vendor
)

target_link_libraries(model yaml-cpp)

target_include_directories(model
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(modelTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(common geom rclcpp yaml_cpp_vendor)

install(
    TARGETS model
    EXPORT modelTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config   DESTINATION share/${PROJECT_NAME})

ament_package()