cmake_minimum_required(VERSION 2.8)
project( main )
find_package( OpenCV REQUIRED )
find_package(nlohmann_json 3.7.0 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )

project(CameraCalibration)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(CameraCalibration CameraCalibration.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(CameraCalibration ${OpenCV_LIBS})

project(CameraPosition)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(CameraPosition CameraPosition.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(CameraPosition ${OpenCV_LIBS})

project(webcamTracking)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(webcamTracking webcamTracking.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(webcamTracking ${OpenCV_LIBS})

project(tcp_client)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(tcp_client tcp_client.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(tcp_client ${OpenCV_LIBS})

project(gui)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.7.0 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED gtk+-3.0)
include_directories(${GTK3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(gui GUI.cpp)  # Replace "process.cpp" with your new source file name
target_link_libraries(gui ${GTK3_LIBRARIES})
target_link_libraries(gui ${OpenCV_LIBS})