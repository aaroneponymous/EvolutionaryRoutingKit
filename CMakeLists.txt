cmake_minimum_required(VERSION 3.10)
project(ProjectName)

# Set C++17 as the standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Source files
set(SOURCES
    src/ant_colony_system.cpp
    # Add other source files here
)

# Executable
add_executable(acs ${SOURCES})

# Include directories specific to the genetic_static target
target_include_directories(acs PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    /home/aaroneponymous/RoutingKit/include/routingkit
    #include this for the library relative paths
    /home/aaroneponymous/RoutingKit/include
)

message(STATUS "Including RoutingKit from: /home/aaroneponymous/RoutingKit/include/routingkit")

# Assuming the library is named libroutingkit.a or libroutingkit.so
find_library(ROUTING_KIT_LIB NAMES routingkit PATHS /home/aaroneponymous/RoutingKit/lib NO_DEFAULT_PATH)
if(ROUTING_KIT_LIB)
    message(STATUS "RoutingKit library found: ${ROUTING_KIT_LIB}")
    target_link_libraries(acs ${ROUTING_KIT_LIB})
else()
    message(STATUS "RoutingKit library not found. Ensure RoutingKit is correctly installed or the path is set correctly.")
endif()


# If RoutingKit is header-only and uses templates heavily, no need to link against a library
