# LiDAR-SLAM-code-comments

origin repository: https://github.com/gaoxiang12/slam_in_autonomous_driving

## CMakeLists.txt中的一些设置
### include(cmake_modules/CompilerOptions.cmake)的
在cmake_modules/CompilerOptions.cmake中定义了一些编译选项，这些选项可以根据不同的编译器和平台进行调整。通过在CMakeLists.txt中包含这个文件，可以统一管理和应用这些编译选项，确保项目在不同环境下具有一致的编译行为。  

- 它的功能是为每个target统一设置如下编译选项：
  - C++17标准
  - 编译器警告和错误设置
  - 跨平台兼容性处理
  - 包含目录配置
- 使用方法：
  ```cmake
  # 包含函数定义
  include(../cmake_modules/CompilerOptions.cmake)
    
  # 创建target
  add_library(feature3d ...)
    
  # 应用统一属性
  set_global_target_properties(feature3d)
  ```
  
### CMAKE_EXPORT_COMPILE_COMMANDS ON：

✅ 开发友好：支持现代IDE和工具链
✅ 调试方便：查看具体编译命令
✅ 质量控制：集成静态分析工具

### CMAKE_POSITION_INDEPENDENT_CODE ON：
✅ 安全增强：支持ASLR
✅ 灵活部署：支持共享库和插件
✅ 现代标准：符合当前最佳实践

### ccache
ccache（Compiler Cache）是一个编译器缓存工具，它：
- 缓存编译结果：当相同文件再次编译时，直接使用缓存
- 大幅加速：第二次及之后的编译通常快5-100倍
- 完全透明：开发者不需要改变编译流程
工作原理：
```
# 没有ccache
g++ -c file.cpp -o file.o

# 有ccache
ccache g++ -c file.cpp -o file.o
   ↓
ccache检查缓存密钥（源文件+编译选项+编译器版本等）
   ↓
缓存命中 → 直接复制缓存的.o文件
缓存未命中 → 真正编译，结果存入缓存
```