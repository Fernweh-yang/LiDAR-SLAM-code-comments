//
// Created by xiang on 2021/8/18.
//

#include "ch5/bfnn.h"
#include <execution>

namespace sad {
// ********* 单点暴力近邻，单线程 *********
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    // 用匿名函数计算单点和目标点云所有点的距离
    // 用std::min_element找出最小值
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}

// ********* 单点暴力K近邻，单线程 *********
std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k) {
    struct IndexAndDis {
        IndexAndDis() {}
        IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
        int index_ = 0;
        double dis2_ = 0;
    };

    std::vector<IndexAndDis> index_and_dis(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        index_and_dis[i] = {i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
    }
    // 计算完所有距离后，用排序算法找出k个最近的
    std::sort(index_and_dis.begin(), index_and_dis.end(),
              [](const auto& d1, const auto& d2) { return d1.dis2_ < d2.dis2_; });
    std::vector<int> ret;
    std::transform(index_and_dis.begin(), index_and_dis.begin() + k, std::back_inserter(ret),
                   [](const auto& d1) { return d1.index_; });
    return ret;
}

// ********* 点云暴力近邻，多线程 *********
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    // 使用 C++17 标准库中的并行算法（std::for_each）来实现多线程并行化
    // 并行的充索引向量
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    // std::execution::par_unseq 执行策略，允许并行和无序执行
    // 并行的计算每个点2：second在点1:first中最近的点
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

// ********* 点云暴力近邻，单线程 *********
void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 单线程版本
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    // std::execution::seq是std::for_each的单线程策略
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

// ********* 点云暴力k近邻，多线程 *********
void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化for_each
    matches.resize(index.size() * k);
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        // 调用单点暴力K近邻，单线程 
        auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
        for (int i = 0; i < v.size(); ++i) {
            matches[idx * k + i].first = v[i];
            matches[idx * k + i].second = idx;
        }
    });
}

}  // namespace sad
