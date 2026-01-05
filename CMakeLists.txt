cmake_minimum_required(VERSION 3.16)  # 提高版本以获得更好特性
project(slam_in_auto_driving VERSION 1.0.0 LANGUAGES CXX)

# 顶层CMakeLists.txt（只做 3 件事）
# ========== 1.构建选项 ==========
# 策略设置：CMP007策略可以保证option中的参数如果在命令行中传递进来不会被默认值覆盖
cmake_policy(SET CMP0077 NEW)
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# ========== 2. 全局配置（只配置一次）==========
# 2.1设置C++标准 - 更现代的方式
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求编译器必须支持指定的C++标准
set(CMAKE_CXX_EXTENSIONS OFF)       # 禁用编译器特定的扩展，确保代码在不同编译器间的可移植性
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)           # 生成compile_commands.json文件，包含项目中每个源文件的完整编译命令。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)         # 启用位置无关代码（Position Independent Code, PIC），让生成的代码可以在内存任意位置执行。

# 2.2添加自定义的CMake模块
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules
)
message(STATUS "当前源码路径: ${CMAKE_CURRENT_SOURCE_DIR}")
# include等价于在这里 “粘贴并执行” CheckCXXCompilerFlag和CompilerOptions.cmake 的内容。
include(CheckCXXCompilerFlag)       # 提供检查编译器标志的机制
include(CompilerOptions)      # 自定义的集中管理项目的编译器设置
include(Dependencies)               # 自定义的第三方库连接

# 2.3 集成ccache编译缓存工具，可以显著加快项目的编译速度。
if (USE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)  # 在系统中查找ccache
    if (CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
        message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    endif ()
endif ()

# ========== 3.子模块 ==========
# 添加需要编译的对象
## 依赖的子目录
add_subdirectory(thirdparty/livox_ros_driver)
add_subdirectory(src/ch2)
add_subdirectory(src/common)
add_subdirectory(src/tools)
