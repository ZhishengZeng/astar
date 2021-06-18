cmake_minimum_required(VERSION 3.14...3.16)
set(CMAKE_CXX_STANDARD 11)
project(ASTAR VERSION 0.0.1
LANGUAGES CXX)

include_directories(
${PROJECT_SOURCE_DIR}/include
)

add_library(astar 
${PROJECT_SOURCE_DIR}/src/Map.cpp
)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")


add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(main astar)