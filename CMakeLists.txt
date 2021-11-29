cmake_minimum_required(VERSION 3.0)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Debug)

# default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS CMAKE_BUILD_TYPE ": ${CMAKE_BUILD_TYPE}")
  add_compile_options(-O3)
else(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS CMAKE_BUILD_TYPE ": ${CMAKE_BUILD_TYPE}")
  add_compile_options(-g)
  add_compile_options(-O0)
endif(CMAKE_BUILD_TYPE MATCHES Release)

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Library Status: ")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3:")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
)

add_executable(test_opticalflow
  ./test_opticalflow.cc
  ./opticalflow.cc
)
target_link_libraries(test_opticalflow
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
)