cmake_minimum_required(VERSION 2.8)
#set(OpenCV_DIR "/usr/local/opencv3/share/OpenCV")
#set(OpenCV_CONFIG_PATH "/usr/local/opencv3/share/OpenCV")
set(CMAKE_BUILD_TYPE "Release")

project(scout)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable( scout ${sourcefiles} )
target_link_libraries( scout "/opt/intel/compilers_and_libraries_2016.3.210/linux/compiler/lib/intel64/libimf.so" pthread ${OpenCV_LIBS})
 



