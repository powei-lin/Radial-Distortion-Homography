cmake_minimum_required(VERSION 3.0.0)
project(multiCameraVO VERSION 0.1.0)

include(CTest)
enable_testing() 

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third-party/ethz_apriltag2/include)

file(GLOB APRILTAG_SRCS ${PROJECT_SOURCE_DIR}/third-party/ethz_apriltag2/src/*.cc)
add_library(apriltags 
  ${APRILTAG_SRCS})
target_link_libraries (apriltags ${OpenCV_LIBS} Eigen3::Eigen)


add_executable(homography ${PROJECT_SOURCE_DIR}/src/homography.cpp)
target_link_libraries(homography 
    ${OpenCV_LIBS} 
    ${CERES_LIBRARIES} 
    Eigen3::Eigen 
    apriltags)

add_executable(test_eigen ${PROJECT_SOURCE_DIR}/src/test_eigen.cpp)
target_link_libraries(test_eigen 
    ${OpenCV_LIBS} 
    ${CERES_LIBRARIES} 
    Eigen3::Eigen 
    apriltags)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
