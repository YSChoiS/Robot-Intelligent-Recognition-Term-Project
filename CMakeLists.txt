cmake_minimum_required(VERSION 2.8.3)
project(RIR_Term_Project)

# Build the talker and listener. Each one uses the following ROS packages,
# which we need to find_package() individually:
#   roscpp (the client library)
#   std_msgs (contains the std_msgs/String message type)
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# We've found them; now use their variables in the usual way to configure
# the compile and link steps.
# Note: we skip calling link_directories() because ROS packages follow the
# recommended CMake practice of returning absolute paths to libraries
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(termp1_1 termp1_1.cpp)
target_link_libraries(termp1_1 ${roscpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(termp1_2 termp1_2.cpp)
target_link_libraries(termp1_2 ${roscpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(Controller Controller.cpp)
add_subdirectory(Gamepad)
target_link_libraries(Controller CONSOLE::Gamepad ${roscpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

# (optional) Install the executables.
install(TARGETS termp1_1
        DESTINATION bin)