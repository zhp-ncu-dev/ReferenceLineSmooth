cmake_minimum_required(VERSION 3.6)
project(qp_spline_smoothing)

set(CMAKE_CXX_STANDARD 11)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB_RECURSE SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(qp_spline_smoothing ${SRC})


find_package(PythonLibs 2.7)
target_include_directories(qp_spline_smoothing PRIVATE ${PYTHON_INCLUDE_DIRS})

set(ENV_QPOASES_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/qpoases/include")
file(GLOB ENV_QPOASES_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/qpoases/lib/Debug/*.a)
include_directories(${ENV_QPOASES_INCLUDE})
link_libraries(${ENV_QPOASES_LIB})

target_link_libraries(qp_spline_smoothing ${PYTHON_LIBRARIES} ${ENV_QPOASES_LIB})
