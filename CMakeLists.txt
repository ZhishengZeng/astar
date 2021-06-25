cmake_minimum_required(VERSION 3.11)
project(ASTAR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
${PROJECT_SOURCE_DIR}/include
)

add_library(astar 
${PROJECT_SOURCE_DIR}/src/Model.cpp
${PROJECT_SOURCE_DIR}/src/Util.cpp
)

# SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")


add_executable(answer ${PROJECT_SOURCE_DIR}/tests/answer.cpp)
add_executable(random ${PROJECT_SOURCE_DIR}/tests/random.cpp)
target_link_libraries(answer astar)
target_link_libraries(random astar)