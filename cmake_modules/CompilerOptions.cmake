# 这个cmake函数的功能是为“某一个 target”统一设置：
  # C++ 标准
  # 编译警告等级
  # 编译器差异（GCC / Clang / MSVC）
  # 公共 include 目录
  # 必要的宏定义
# 使用方式：
  # add_library(ch2_core ...)或者 add_executable(ch2_core main.cpp)
  # set_global_target_properties(ch2_core)
function(set_global_target_properties target)
  # 使用现代CMake方式设置C++17标准，PUBLIC表示依赖此target的其他target也会继承C++17要求
  target_compile_features(${target} PUBLIC cxx_std_17)
  # 跨平台兼容性处理：在MSVC编译器下定义_USE_MATH_DEFINES宏，确保Windows上能使用M_PI等数学常量
  target_compile_definitions(${target} PUBLIC $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:_USE_MATH_DEFINES>)
  # 设置编译器特定的额警告、错误处理选项，提升代码质量
  target_compile_options(
    ${target}
    PRIVATE # MSVC(windows)
            $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/W4>  # 最高警告级别
            $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/WX>  # 视警告为错误（严格模式）
            # Clang/AppleClang(macOS/ios)
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-fcolor-diagnostics>  # 彩色诊断输出
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Werror>              # 视警告为错误
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wall>                # 启用所有警告
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wextra>              # 启用所有额外警告
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wconversion>         # 检查隐式类型转换
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wno-sign-conversion> # 忽略有符号/无符号转换警告
            # GNU(Linux)
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fdiagnostics-color=always>        # 彩色输出
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Werror>                           # 视警告为错误，严格模式
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall>                             # 全面警告
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wextra>                           # 全面警告
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-pedantic>                         # 全面警告
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-align>                      # 指针对齐
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-qual>                       # 常量限定
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wconversion>                      # 类型转换
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wdisabled-optimization>           # 被禁用的优化
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Woverloaded-virtual>)             # 虚函数重载问题
  set(INCLUDE_DIRS ..)
  get_filename_component(INCLUDE_DIRS ${INCLUDE_DIRS} PATH)
  target_include_directories(${target} PRIVATE ../components/kiss_icp/kiss_icp/cmake
                             PUBLIC $<BUILD_INTERFACE:${INCLUDE_DIRS}> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
endfunction()
