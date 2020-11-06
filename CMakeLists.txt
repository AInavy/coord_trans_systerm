cmake_minimum_required(VERSION 2.8)
project(KLT)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#opencv
set(OpenCV_DIR /home/mahj/software/opencv-3.3.0/build)
find_package(OpenCV REQUIRED)

#boost
find_package(Boost REQUIRED COMPONENTS system program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
list(APPEND boost_lib ${Boost_SYSTEM_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})

set(PROJECT_ROOT ".")
list(APPEND proj4 /usr/local/lib/libproj.so)

include_directories(${PROJECT_ROOT}/include)
include_directories(/usr/local/include)

set(SRC_ROOT ${PROJECT_ROOT}/src)

file(GLOB src_files ${SRC_ROOT}/*.cpp)

add_executable(main main.cpp ${src_files})
target_link_libraries(main ${OpenCV_LIBS} ${proj4} ${boost_lib})


