# ============================================================
# Dependencies.cmake 整个工程第三方库的依赖管理
# 子模块cmakelists.txt使用方法：
# 用到全部：
# add_executable(run src/main.cpp)
# target_link_libraries(run
# 	PRIVATE
# 	project_deps
# )
# 用到部分：
# target_link_libraries(mapping
# 	PRIVATE
# 	Eigen3::Eigen
# 	g2o
# 	OpenCV::opencv_core
# )
# ============================================================
# 作用：这个 cmake 文件，在整个 CMake 配置生命周期内，只能执行一次
include_guard(GLOBAL)

# ========== Spdlog ==========
find_package(spdlog REQUIRED)

# ========== Eigen ==========
find_package(Eigen3 REQUIRED)

# ========== OpenCV ==========
find_package(OpenCV REQUIRED)

# ========== PCL ==========
find_package(PCL REQUIRED)

# ========== Glog ==========
find_package(Glog REQUIRED)

# ========== GFlags ==========
find_package(gflags REQUIRED)

# ========== yaml-cpp ==========
find_package(yaml-cpp REQUIRED)

# ========== Pangolin ==========
find_package(Pangolin REQUIRED)

# ========== TBB (oneAPI) ==========
find_package(TBB REQUIRED)

# ========== ROS ==========
find_package(catkin REQUIRED COMPONENTS
	roscpp rospy std_msgs sensor_msgs pcl_ros pcl_conversions angles
)

# ========== Sophus ==========
add_library(sophus INTERFACE)
target_include_directories(sophus
	INTERFACE
		${PROJECT_SOURCE_DIR}/thirdparty/sophus
)

# ========== g2o（thirdparty）==========
add_library(g2o INTERFACE)
# 作用：任何 link 到 g2o 的 target，都会自动获得这个 include 路径
# target_include_directories:解决“编译期找头文件” 即#include <xxx.h> 去哪里找
target_include_directories(g2o
	INTERFACE
		${PROJECT_SOURCE_DIR}/thirdparty/g2o
)
# target_link_libraries:解决“链接期找符号 + 传递依赖”
# .so: 动态链接库（shared object）
# g2o 不是一个库，而是一“组”库。每个 .so 负责 不同层级 / 不同功能的符号。
# 任何target_link_libraries到g2o的子模块，都会自动链接到这些.so
target_link_libraries(g2o
	INTERFACE
		g2o_core
		g2o_stuff
		g2o_solver_dense
		g2o_solver_csparse
		g2o_csparse_extension
		g2o_types_sba
)

# ========== 聚合一个总依赖 ==========
add_library(project_deps INTERFACE)
target_link_libraries(project_deps
	INTERFACE
		Eigen3::Eigen
		sophus
		g2o
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		${catkin_LIBRARIES}
		glog
		gflags
		yaml-cpp
        ${Pangolin_LIBRARIES}
		TBB::tbb
)
# Ensure message-generated headers (catkin devel include) are available to all targets
target_include_directories(project_deps
	INTERFACE
		${CMAKE_BINARY_DIR}/devel/include
)