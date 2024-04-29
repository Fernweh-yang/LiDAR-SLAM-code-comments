/// 本节程序演示一个正在作圆周运动的车辆
#include <gflags/gflags.h>  // 程序参数管理工具
#include <glog/logging.h>   // 日志管理工具

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 车辆的角速度与线速度可以在flags中设置
// 定义命令行参数
DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);                 // 初始化日志类，argv[0]是程序的名称
    FLAGS_stderrthreshold = google::INFO;               // 设置了 glog 输出到标准错误流的最低日志级别为 INFO。这意味着只有 INFO 级别及以上的日志消息才会被输出到标准错误流。
    FLAGS_colorlogtostderr = true;                      // 在输出到标准错误流时否启用彩色日志。
    google::ParseCommandLineFlags(&argc, &argv, true);  // 解析命令行参数并初始化相应的 gflags 变量, true参数表示如果有不符合规范的参数打印出错误信息。

    /// 可视化
    // 使用tools中的pangolin_window
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    const double dt = 0.05;                                                      // 每次更新的时间

    while (ui.ShouldQuit() == false) {
        // 更新自身位置
        Vec3d v_world = pose.so3() * v_body;    // 世界坐标系下的速度
        pose.translation() += v_world * dt;     // 更新位姿的位移部分

        // 更新自身旋转
        if (FLAGS_use_quaternion) {
            // 四元数下的旋转更新
            // 笔记公式2.2.13
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            // SO3下的旋转更新
            // 笔记公式：2.1.3
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }
    
        LOG(INFO) << "pose: " << pose.translation().transpose();    // 将一条信息(INFO级别)记录到日志中，glog默认输出到终端
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);   // usleep微秒为单位，1秒=1e6w微妙
    }

    ui.Quit();
    return 0;
}