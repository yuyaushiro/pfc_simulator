cmake_minimum_required(VERSION 2.8)
project(pfc_simulator)

add_compile_options("-std=c++14")

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(GLEW REQUIRED)
find_package(GLFW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PNG REQUIRED)

include_directories(${GLEW_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})
include_directories(${PNG_INCLUDE_DIRR})
include_directories(include)

link_directories(${PNG_LIBRARY_DIRS})

add_executable(simulator.out
  src/main.cpp
  src/avoidance.cpp
  src/circle.cpp
  src/cmd_vel.cpp
  src/goal.cpp
  src/line.cpp
  src/mcl.cpp
  src/particle.cpp
  src/pfc.cpp
  src/point.cpp
  src/pose.cpp
  src/robot.cpp
  src/state_space.cpp
  src/state.cpp
  src/window.cpp
)
target_link_libraries(simulator.out ${GLFW_LIBRARIES} ${OPENGL_LIBRARY} ${GLEW_LIBRARIES} ${PNG_LIBRARY})
