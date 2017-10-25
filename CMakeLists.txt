cmake_minimum_required(VERSION 2.8.4)
project(SerialMonitor)

set(CMAKE_C_STANDARD 99)
FIND_PACKAGE (Threads)

set(SOURCE_FILES main.c jRead.c )
add_executable(SerialMonitor ${SOURCE_FILES})
target_link_libraries(SerialMonitor ${CMAKE_THREAD_LIBS_INIT})