cmake_minimum_required(VERSION 3.16)
project(ixy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-g -O2 -march=native -fomit-frame-pointer
    -D_XOPEN_SOURCE=700
    -D_DEFAULT_SOURCE
    -Wall
    -Wextra
    -Wpedantic
)

file(GLOB_RECURSE IXY_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(main ${IXY_SOURCES})
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)