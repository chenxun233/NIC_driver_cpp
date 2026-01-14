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

set(DRIVER_SOURCES
    src/basic_dev.cpp
    src/basic_ring_buffer.cpp
    src/dma_memory_allocator.cpp
    src/factory.cpp
    src/ixgbe_ring_buffer.cpp
    src/memory_pool.cpp
    src/vfio_dev.cpp
)

add_executable(test_app_loopsend ${DRIVER_SOURCES} src/test_app_loopsend.cpp)
target_include_directories(test_app_loopsend PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(test_app_pcap ${DRIVER_SOURCES} src/test_app_pcap.cpp)
target_include_directories(test_app_pcap PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)