cmake_minimum_required(VERSION 3.5)
project(slam_in_auto_driving)

set(CMAKE_BUILD_TYPE "Release")                                     # 设置构建类型为 Release，即优化编译选项，相比于debug模式去除了调试信息
set(CMAKE_CXX_STANDARD 17)                                          # c++标准
set(CMAKE_CXX_FLAGS "-w")                                           # 设置 C++ 编译器的警告等级，这里是关闭所有警告。
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")      # 设置 Release 模式下的编译选项：二级优化(-O2)，生成调试信息(-g)，更详细的调试信息用于GDB(--ggdb),以及附加的编译器标志。
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")                  # 设置 Debug 模式下的编译选项

add_definitions("-DCATKIN_ENABLE_TESTING=0")                        # 添加预处理器定义，这里是不适用ROS编译工具catkin的test。即catkin make后不会运行catkin make test

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)       # 设置可执行文件的输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)       # 设置库文件的输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)       # 设置静态库文件的输出目录

option(BUILD_WITH_UBUNTU1804 OFF)                                   # 定义一个变量设置它为off

# PROJECT_SOURCE_DIR返回的是最顶一级CMakeLists.txt所在的目录
include(cmake/packages.cmake)                                       # 包含一个额外的 CMake 文件，用于配置项目的依赖包。
include_directories(${PROJECT_SOURCE_DIR}/src)                      # 在src目录中寻找头文件xx.h, 确保我们源文件中用到的头文件被连接
include_directories(${PROJECT_SOURCE_DIR})                          # 在当前目录中寻找头文件xx.h
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)      # 在include目录中寻找头文件

add_subdirectory(src)                                               # 让cmake也处理src文件夹其他的CMakeLists.txt 
