
cmake_minimum_required(VERSION 3.10)
project(sync_read_write LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(dynamixel_sdk REQUIRED)

add_executable(pos_reader pos_reader.cpp)
target_link_libraries(pos_reader dynamixel_sdk::dynamixel_sdk)

