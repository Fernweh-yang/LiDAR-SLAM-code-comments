//
// Created by xiang on 2022/1/4.
//

#include "ch3/utm_convert.h"
#include "common/math_utils.h"
#include "utm_convert/utm.h"

#include <glog/logging.h>

namespace sad {

// ! 使用开源库utm_convert/utm.h将经纬度转为UTM坐标
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

// ! 使用开源库utm_convert/utm.h将UTM坐标转为经纬度
bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0],
                                       utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= math::kRAD2DEG;
    return ret == 0;
}

// ! 将utm坐标从gnss坐标系转为车身坐标系
bool ConvertGps2UTM(GNSS& gps_msg, const Vec2d& antenna_pos, const double& antenna_angle, const Vec3d& map_origin) {
    // * 1. 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk.z_ = gps_msg.lat_lon_alt_[2];

    // * 2. GPS heading朝向 转成弧度
    // math::kDEG2RAD = pi/180
    // 在这里方向角heading_valid_是因为双天线方案所以才可以得到的
    // 我们使用东北天坐标系的utm，但该rtk厂商输出的是北东地坐标系，两者旋转方向相反，因此需要将朝向角转换一下。
    double heading = 0;
    if (gps_msg.heading_valid_) {
        heading = (90 - gps_msg.heading_) * math::kDEG2RAD;  // 北东地转到东北天
    }

    // * 3. 得到gnss相对于车身的坐标TBG
    // 这里的安装偏角antenna_angle 和 安装偏移antenna_pos 都是依靠装gnss时标定得到的
    SE3 TBG(SO3::rotZ(antenna_angle * math::kDEG2RAD), Vec3d(antenna_pos[0], antenna_pos[1], 0));
    SE3 TGB = TBG.inverse();

    // * 4. 由gnss的utm坐标确定车身本体的世界坐标TWB
    /// 若指明地图原点，则减去地图原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];
    SE3 TWG(SO3::rotZ(heading), Vec3d(x, y, z));    // gnss的世界坐标是根据 朝向 + utm 得到的
    SE3 TWB = TWG * TGB;

    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_[0] = TWB.translation().x();
    gps_msg.utm_.xy_[1] = TWB.translation().y();
    gps_msg.utm_.z_ = TWB.translation().z();

    if (gps_msg.heading_valid_) {
        // 组装为带旋转的位姿
        gps_msg.utm_pose_ = TWB;
    } else {
        // 组装为仅有平移的SE3
        // 注意当安装偏移存在时，并不能实际推出车辆位姿
        gps_msg.utm_pose_ = SE3(SO3(), TWB.translation());
    }

    return true;
}

bool ConvertGps2UTMOnlyTrans(GNSS& gps_msg) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk);
    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_ = utm_rtk.xy_;
    gps_msg.utm_.z_ = gps_msg.lat_lon_alt_[2];
    gps_msg.utm_pose_ = SE3(SO3(), Vec3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_));
    return true;
}

}  // namespace sad