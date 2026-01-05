# ========== 静态库：tools ==========
# STATIC 创建一个静态库.a
# 被link时，直接把.a的内容copy到可执行文件中
# 适合小型库，减少运行时依赖
add_library(${PROJECT_NAME}.tools STATIC
        pointcloud_convert/velodyne_convertor.cc
        pointcloud_convert/packets_parser.cc
        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_trajectory.cc
        ui/ui_cloud.cc
        )
# PUBLIC:当前 target 可见，同时 传播给 link 到这个 target 的使用者
# PRIVATE:当前 target 可见，但不传播给使用者
# INTERFACE:当前 target 不可见，但传播给使用者
target_include_directories(${PROJECT_NAME}.tools
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/tools
        ${PROJECT_SOURCE_DIR}/thirdparty
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}.tools
    PUBLIC
        project_deps
)


# ========== 可执行文件：ui_test ==========
add_executable(ui_test ui/ui_test.cc)

target_link_libraries(ui_test
    PRIVATE
        ${PROJECT_NAME}.tools
        project_deps
)
