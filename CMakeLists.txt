cmake_minimum_required(VERSION 2.8)
project( main )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )

# Add a new project called "ProcessImage"
project(CameraCalibration)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(CameraCalibration CameraCalibration.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(CameraCalibration ${OpenCV_LIBS})