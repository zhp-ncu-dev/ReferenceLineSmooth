cmake_minimum_required(VERSION 3.6)
project(discrete_point_smoothing)

set(CMAKE_CXX_STANDARD 11)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
file(GLOB_RECURSE SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")

add_executable(discrete_point_smoothing ${SRC})


# 添加 ipopt 库
set(ENV_IPOPT_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/ipopt/include/coin")
file(GLOB ENV_IPOPT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/ipopt/lib/*.so)
include_directories(${ENV_IPOPT_INCLUDE})
link_libraries(${ENV_IPOPT_LIB})

# 添加 adolc 库
set(ENV_ADOLC_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/adolc/include")
file(GLOB ENV_ADOLC_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/adolc/lib64/*.so)
include_directories(${ENV_ADOLC_INCLUDE})
link_libraries(${ENV_ADOLC_LIB})

# 添加 ColPack 库
set(ENV_COLPACK_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/colpack/include/ColPack")
file(GLOB ENV_COLPACK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/colpack/lib/*.so)
include_directories(${ENV_COLPACK_INCLUDE})
link_libraries(${ENV_COLPACK_LIB})

# 添加 proj4 库
set(ENV_PROJ4_INCLUDE  "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/proj4/include")
file(GLOB ENV_PROJ4_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/proj4/lib/Debug/libproj4_linux_x64.a)
include_directories(${ENV_PROJ4_INCLUDE})
link_libraries(${ENV_PROJ4_LIB})

# 添加 osqp 库
set(ENV_OSQP_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/osqp/include")
file(GLOB ENV_OSQP_LIB ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/osqp/lib/*.so)
include_directories(${ENV_OSQP_INCLUDE})
link_libraries(${ENV_OSQP_LIB})

find_package(PythonLibs 2.7)
target_include_directories(discrete_point_smoothing PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(discrete_point_smoothing
        ${ENV_IPOPT_LIB}
        ${ENV_ADOLC_LIB}
        ${ENV_COLPACK_LIB}
        ${ENV_PROJ4_LIB}
        ${ENV_OSQP_LIB}
        ${PYTHON_LIBRARIES})


