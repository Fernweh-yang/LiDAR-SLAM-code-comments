// ! 单纯依靠笔记公式2.1.3对IMU数据做2次积分，得到运动物体的位姿 
#include <glog/logging.h>   // 日志管理工具
#include <iomanip>  // c++标准库：主要用于格式化输入输出操作，例如设置宽度、精度、填充字符、调整字段

#include "ch3/imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    // TxtIO类读取了txt文件
    // * 该类使用std::function内置了针对imu,odom,gnss三种数据的回调函数
    // 后面想处理哪种数据，就把处理函数或lambda表达式传入这些回调函数即可
    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);                                  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);    // 陀螺仪的邻偏 
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);             // 加速度计的邻偏

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);   // 用于对IMU做积分的类

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;      // 可视化
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    // * 定义一个lambda表达式(匿名函数)
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v, const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { 
            fout << v[0] << " " << v[1] << " " << v[2] << " "; 
        };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);                         // IMU的位置
        save_quat(fout, R.unit_quaternion());       // IMU的旋转矩阵
        save_vec3(fout, v);                         // IMU的速度
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/state.txt");
    // * 再定义一个lambda表达式(匿名函数)作为上面TxtIO::io的回调函数
    // 这个匿名函数的参数imu，在Go()中调用此回调函数时被赋予
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go();

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}