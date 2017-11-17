cmake_minimum_required(VERSION 2.8.4)
project(SerialMonitor)

set(CMAKE_C_STANDARD 99)

# specify the cross compiler
#set(CMAKE_C_COMPILER   /usr/local/arm-2009q3/bin/arm-none-linux-gnueabi-gcc)
#set(CMAKE_CXX_COMPILER   /usr/local/arm-2009q3/bin/arm-none-linux-gnueabi-g++)
#add_compile_options(-std=gnu99)
#set(CMAKE_FIND_ROOT_PATH /usr/local//arm-none-linux-gnueabi/libc/armv4t)

# search for programs in the build host directories
#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#$set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

#include_directories(
#        /usr/local/arm-2009q3/arm-none-linux-gnueabi/libc/include
#        ./include
#)
#link_directories(
#        /usr/local/arm-2009q3/arm-none-linux-gnueabi/libc/lib
#)

FIND_PACKAGE (Threads)

set(SOURCE_FILES main.c jRead.c queue_list.c)
add_executable(SerialMonitor ${SOURCE_FILES})
target_link_libraries(SerialMonitor ${CMAKE_THREAD_LIBS_INIT})