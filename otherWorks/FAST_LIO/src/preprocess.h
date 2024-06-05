#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
// 枚举类型：表示特征点的类型
// 正常点，可能平面点，确定平面点，有跳跃的边缘特征点(台阶)， 平面的边缘特征点(地板墙)， 线段， 无效点
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
// 位置标识？
// 前一个/后一个
enum Surround{Prev, Next};
// 有跳跃的边缘特征类型？
// 正常，0,180,无穷大，盲区
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

// 存储激光雷达点的一些属性
struct orgtype
{
  double range;             // 点云在xy平面离雷达中心的距离
  double dista;             // 当前点与后一个点之间的距离
  
  // 假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];          // 这个是角OAM和角OAN的cos值
  double intersect;         // 这个是角MAN的cos值
  E_jump edj[2];            // 前后两点的类型
  Feature ftype;            // 点类型

  // 构造函数
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;            //默认为正常点
    intersect = 2;
  }
};

// * velodyne数据结构
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                  // PCL) 中的一个宏，用于将 Eigen::Vector4f 类型的 4D 向量（x, y, z, w）添加到自定义点云结构体中
      float intensity;
      float time;
      uint16_t ring;                    // 点所属的圈数
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen库的内存对齐
  };
}  // namespace velodyne_ros
// POINT_CLOUD_REGISTER_POINT_STRUCT 是 Point Cloud Library (PCL) 提供的一个宏。
// 用于将自定义的点云结构注册到 PCL 中，使其可以与 PCL 的各种处理算法兼容。
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)   // 参数1：类型 2： 结构体成员名称 3：pcl内部使用的字段名称
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

// * ouster数据结构
namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;                  // 强度
      uint32_t t;
      uint16_t reflectivity;            // 反射率
      uint8_t  ring;
      uint16_t ambient;                 // 没用到
      uint32_t range;                   // 距离
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// ! 用于对激光雷达点云数据进行预处理
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  // 对Livox自定义Msg格式的激光雷达数据进行处理   
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // 对ros的Msg格式的激光雷达数据进行处理
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf; // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;  // 雷达类型、采样间隔、扫描线数、扫描频率
  double blind;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
