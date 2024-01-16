cmake_minimum_required(VERSION 3.10)
project(msckf)
set(CMAKE_CXX_STANDARD 17)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(HEADER_FILES
    include/msckf.hpp
    include/matrix_utils.hpp
    include/types.hpp
)

pybind11_add_module(msckf src/bindings.cpp ${HEADER_FILES})

target_link_libraries(msckf PUBLIC)
