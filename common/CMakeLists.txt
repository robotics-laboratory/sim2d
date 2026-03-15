cmake_minimum_required(VERSION 3.22)

project(common)

find_package(ament_cmake REQUIRED)

add_library(common SHARED
    src/format.cpp
)

target_include_directories(common
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_export_targets(commonTargets HAS_LIBRARY_TARGET)

install(
    TARGETS common
    EXPORT commonTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()