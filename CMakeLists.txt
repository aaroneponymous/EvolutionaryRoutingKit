cmake_minimum_required(VERSION 3.22)
project(ACS)

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


# # If RoutingKit is header-only and uses templates heavily, no need to link against a library

# RoutingKit Paths
# set(ROUTINGKIT_INCLUDE_DIR "/home/aaroneponymous/RoutingKit/include/routingkit")
# set(ROUTINGKIT_LIBRARY_DIR "/home/aaroneponymous/RoutingKit/lib")
# set(ROUTINGKIT_LIBRARY_NAME "libroutingkit.so")

# # Add RoutingKit include directory
# include_directories(${ROUTINGKIT_INCLUDE_DIR})

# # Find and link the appropriate RoutingKit Library
# find_library(ROUTINGKIT_LIB NAMES ${ROUTINGKIT_LIBRARY_NAME} PATHS ${ROUTINGKIT_LIBRARY_DIR})
# if(ROUTINGKIT_LIB)
#     message(STATUS "RoutingKit library found: ${ROUTINGKIT_LIB}")
#     target_link_libraries(acs ${ROUTINGKIT_LIB})
# else()
#     message(STATUS "RoutingKit library not found. Ensure RoutingKit is correctly installed or the path is set correctly.")
# endif()

# set_target_properties(acs PROPERTIES INSTALL_RPATH ${ROUTINGKIT_LIBRARY_DIR})


# LibXL PPaths
set(LIBXL_INCLUDE_DIR "/home/aaroneponymous/External_Lib/libxl-4.3.0.14/include_cpp")
set(LIBXL_LIBRARY_DIR "/home/aaroneponymous/External_Lib/libxl-4.3.0.14/lib64")  
set(LIBXL_LIBRARY_NAME "libxl.so")

# Add LibXL include directory
include_directories(${LIBXL_INCLUDE_DIR})

# Find and link the appropriate LibXL library
find_library(LIBXL_LIB NAMES ${LIBXL_LIBRARY_NAME} PATHS ${LIBXL_LIBRARY_DIR})
if (LIBXL_LIB)
    message(STATUS "LibXL library found: ${LIBXL_LIB}")
    target_link_libraries(acs ${LIBXL_LIB})
else()
    message(STATUS "LibXL library not found. Ensure the path is correct and the library is available.")
endif()

# Ensure runtime linker finds the library by setting rpath
set_target_properties(acs PROPERTIES INSTALL_RPATH ${LIBXL_LIBRARY_DIR})
