cmake_minimum_required(VERSION 3.23)

project(homography_calculation)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(homography_calculation main.cpp)
target_link_libraries(homography_calculation ${OpenCV_LIBS})