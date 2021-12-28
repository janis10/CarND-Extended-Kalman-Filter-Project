cmake_minimum_required (VERSION 3.5)
project(ExtendedKF LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED NO_MODULE)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(src_folder "src")
set(header_folder "include")

set(sources 
    ${src_folder}/main.cpp 
    ${src_folder}/tools.cpp 
    ${src_folder}/FusionEKF.cpp 
    ${src_folder}/kalman_filter.cpp

    ${header_folder}/tools.h 
    ${header_folder}/FusionEKF.h 
    ${header_folder}/kalman_filter.h
    ${header_folder}/json.hpp
    ${header_folder}/measurement_package.h
    )


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtendedKF ${sources})

target_include_directories(ExtendedKF PRIVATE ${header_folder})

target_link_libraries(ExtendedKF z ssl uv uWS Eigen3::Eigen)