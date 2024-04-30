# -------------- 原始 --------------
# add_executable(motion motion.cc)
# target_link_libraries(motion
#         ${PROJECT_NAME}.common
#         ${PROJECT_NAME}.tools
#         )

# -------------- 尝试独立编译 --------------
cmake_minimum_required(VERSION 3.5)
project(slam_in_auto_driving)

set(CMAKE_BUILD_TYPE "Release")                                     # 设置构建类型为 Release，即优化编译选项，相比于debug模式去除了调试信息
set(CMAKE_CXX_STANDARD 17)                                          # c++标准
set(CMAKE_CXX_FLAGS "-w")                                           # 设置 C++ 编译器的警告等级，这里是关闭所有警告。
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")      # 设置 Release 模式下的编译选项：二级优化(-O2)，生成调试信息(-g)，更详细的调试信息用于GDB(--ggdb),以及附加的编译器标志。
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")                  # 设置 Debug 模式下的编译选项

include_directories(../../)

add_executable(motion motion.cc)
target_link_libraries(motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )