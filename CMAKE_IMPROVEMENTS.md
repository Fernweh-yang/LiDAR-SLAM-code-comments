# CMakeLists.txt 现代化改进建议

## 现状分析

三个CMakeLists.txt文件基本符合现代CMake实践（3.16+），但存在以下改进空间。

---

## 1️⃣ 顶层 CMakeLists.txt 改进

### 当前问题
```cmake
# 缺少默认构建类型设置
# 缺少编译器警告配置
# VERSION 未充分利用
```

### 改进方案
```cmake
cmake_minimum_required(VERSION 3.16)
project(slam_in_auto_driving VERSION 1.0.0 LANGUAGES CXX)

# ========== 1. 全局编译配置 ==========
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
    message(STATUS "Build type set to default: Release")
endif()

# 编译器警告（现代CMake最佳实践）
add_compile_options(
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ========== 2. 版本信息传播 ==========
set(${PROJECT_NAME}_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(${PROJECT_NAME}_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(${PROJECT_NAME}_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# ========== 3. 模块路径 ==========
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# ========== 4. 依赖与编译器选项 ==========
include(CheckCXXCompilerFlag)
include(CompilerOptions)
include(Dependencies)

# ========== 5. 缓存工具 ==========
if(USE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
        message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    endif()
endif()

# ========== 6. 子项目 ==========
add_subdirectory(thirdparty/livox_ros_driver)
add_subdirectory(src/ch2)
add_subdirectory(src/common)
add_subdirectory(src/tools)
```

---

## 2️⃣ src/tools/CMakeLists.txt 改进

### 当前问题
```cmake
# 1. target_link_libraries 中混合了项目库和外部库
# 2. ui_test 应用 PRIVATE 但工具库用 PUBLIC
# 3. include 目录应该从 project_deps 继承
```

### 改进方案
```cmake
# ========== 静态库：tools ==========
add_library(${PROJECT_NAME}.tools STATIC
    pointcloud_convert/velodyne_convertor.cc
    pointcloud_convert/packets_parser.cc
    ui/pangolin_window.cc
    ui/pangolin_window_impl.cc
    ui/ui_car.cc
    ui/ui_trajectory.cc
    ui/ui_cloud.cc
)

# 设置库别名（便于导出）
add_library(${PROJECT_NAME}::tools ALIAS ${PROJECT_NAME}.tools)

# 包含目录：PUBLIC 表示使用此库的目标也需要这些路径
target_include_directories(${PROJECT_NAME}.tools
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/tools>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty>
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)

# 链接：只链接 project_deps（已包含所有必需的库）
target_link_libraries(${PROJECT_NAME}.tools
    PUBLIC
        project_deps
)

# ========== 可执行文件：ui_test ==========
add_executable(ui_test ui/ui_test.cc)

target_link_libraries(ui_test
    PRIVATE
        ${PROJECT_NAME}::tools
        project_deps
)

# 设置输出目录（可选但推荐）
set_target_properties(ui_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
```

### 改进说明
- ✅ 使用 `BUILD_INTERFACE` 区分构建和安装阶段的路径
- ✅ 创建库别名 `::tools` 便于依赖管理
- ✅ 只链接 `project_deps`，避免重复
- ✅ 可执行文件输出到统一的 `bin/` 目录

---

## 3️⃣ src/common/CMakeLists.txt 改进

### 当前问题
```cmake
# 1. 循环依赖：common 依赖 tools（不合理）
# 2. 重复定义 target_include_directories
# 3. 不应混合 ${catkin_LIBRARIES} 和 ${PCL_LIBRARIES}
```

### 改进方案
```cmake
# ========== 消息生成 ==========
add_subdirectory(msg)

# ========== 共享库：common ==========
add_library(${PROJECT_NAME}.common
    io_utils.cc
    timer/timer.cc
    global_flags.cc
    g2o_types.cc
    point_cloud_utils.cc
)

add_library(${PROJECT_NAME}::common ALIAS ${PROJECT_NAME}.common)

# ========== 包含目录 ==========
target_include_directories(${PROJECT_NAME}.common
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/devel/include>  # 消息生成路径
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)

# ========== 链接库 ==========
target_link_libraries(${PROJECT_NAME}.common
    PUBLIC
        project_deps
)

# ========== 消息依赖 ==========
set(MSG_DEPENDENCIES
    monitor_msgs_generate_messages
    velodyne_msgs_generate_messages
    livox_ros_driver_generate_messages
)

add_dependencies(${PROJECT_NAME}.common ${MSG_DEPENDENCIES})

# ========== 编译选项（可选） ==========
target_compile_features(${PROJECT_NAME}.common PRIVATE cxx_std_17)
```

### 改进说明
- ✅ 移除对 `tools` 的依赖（不合理的循环依赖）
- ✅ 合并重复的 `target_include_directories`
- ✅ 统一链接到 `project_deps`
- ✅ 消息依赖清晰化
- ✅ 使用 `BUILD_INTERFACE` 生成器表达式

---

## 4️⃣ 全局改进清单

### 现代CMake最佳实践（应采纳）
- [ ] 使用 `add_library(name::name ALIAS ...)` 创建库别名
- [ ] 使用 `$<BUILD_INTERFACE:>` 和 `$<INSTALL_INTERFACE:>` 区分构建/安装路径
- [ ] 使用 `target_compile_features()` 而非全局 `CMAKE_CXX_STANDARD`
- [ ] 明确 `PUBLIC/PRIVATE/INTERFACE` 的含义
- [ ] 避免重复定义 include 目录
- [ ] 避免循环依赖（common 不应依赖 tools）

### 建议添加（增强可维护性）
```cmake
# Dependencies.cmake 中添加版本检查
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# 添加编译选项统一管理
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 安装规则（便于系统集成）
install(TARGETS ${PROJECT_NAME}.common 
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ 
    DESTINATION include/${PROJECT_NAME}
)
```

---

## 总体评分

| 维度 | 评分 | 评论 |
|------|------|------|
| **现代化程度** | ⭐⭐⭐⭐ | 基础现代，缺少高级特性 |
| **可维护性** | ⭐⭐⭐ | 有重复定义和循环依赖 |
| **最佳实践遵循** | ⭐⭐⭐ | 缺少别名、接口分离 |
| **跨平台兼容性** | ⭐⭐⭐ | 无MSVC特定处理 |
| **可扩展性** | ⭐⭐⭐ | 难以添加新子项目 |

**综合建议：升级为 ⭐⭐⭐⭐⭐**

---

## 参考资源
- CMake 官方最佳实践: https://cmake.org/cmake/help/latest/guide/buildsystem/index.html
- Modern CMake: https://cliutils.gitlab.io/modern-cmake/
