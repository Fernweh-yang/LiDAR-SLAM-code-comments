# ========== 消息生成 ==========
add_subdirectory(msg)

# ========== 共享库：common ==========
# 定义一个 CMake target ${PROJECT_NAME}.common，并用给定源文件构建一个库目标。
add_library(${PROJECT_NAME}.common SHARED
        io_utils.cc
        timer/timer.cc
        global_flags.cc
        g2o_types.cc
        point_cloud_utils.cc
)
# 定义一个 target 在被使用时，编译阶段需要的头文件搜索路径。并通过 PUBLIC 传播给依赖它的目标。
# mplementation（实现）：我这个库自己怎么编译
# Interface（接口）：别人要用我，编译时必须知道什么
# 也就是说在编译自己以及依赖自己的 target 时，需要在哪些目录里查找 #include <...> 的头文件。
# 包含目录：catkin 路径必须是 PUBLIC 以便依赖此库的目标也能找到 sensor_msgs 等
target_include_directories(${PROJECT_NAME}.common
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/thirdparty
        ${catkin_INCLUDE_DIRS}
)
# 定义 target 之间的使用依赖关系，并通过 PUBLIC 传递给下游目标。
# 编译 ${PROJECT_NAME}.common 时, 自动继承 project_deps 的 编译选项和链接库。
# 链接 ${PROJECT_NAME}.common 时, 自动链接 project_deps 的 库文件.so+.a。
# 也就是说${PROJECT_NAME}.common在其 编译、链接以及对外接口中依赖 B 提供的能力。
target_link_libraries(${PROJECT_NAME}.common
    PUBLIC
        project_deps
)

# ========== 消息依赖 ==========
set(sad_messages
        monitor_msgs_generate_messages
        velodyne_msgs_generate_messages
        livox_ros_driver_generate_messages
        )

add_dependencies(${PROJECT_NAME}.common ${sad_messages})