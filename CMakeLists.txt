set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.10)
project(perimeter-threat-classification)

# find OpenCV and PCL
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# include headers
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/fusion/include
)

# build the fusion executable
add_executable(perimeter-threat-classification
  fusion/main.cpp
  fusion/runPipeline.cpp
)

target_link_libraries(perimeter-threat-classification
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
