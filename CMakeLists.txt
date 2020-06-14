PROJECT(Test) 
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_COMPLILER "g++")#设定编译器
SET( CMAKE_BUILD_TYPE Release  )
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(OpenCV_DIR "/home/jlm/jlm/OpenCV4.1.2/Opencv_4.1.2_install_dir/lib/cmake/opencv4") 
find_package(OpenCV REQUIRED) 
include_directories( ${OpenCV_INCLUDE_DIRS} )
#find_package(Threads)	#for pthread library

#include_directories("/usr/include/eigen3")
ADD_LIBRARY( QRBase src/detect.cpp)
TARGET_LINK_LIBRARIES( QRBase
   ${OpenCV_LIBS} )
    
add_executable(QRPositions src/main.cpp)
target_link_libraries(QRPositions 
	${OpenCV_LIBS}
	QRBase
	${CERES_LIBRARIES})

add_executable(Test src/img_test.cpp)
target_link_libraries(Test 
	${OpenCV_LIBS}
	QRBase)
