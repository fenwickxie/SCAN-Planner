// Include Files
#pragma once
#include <math.h>
#include <cmath>
#include "ikd-Tree/ikd_Tree.h"
#include <Eigen/Core>
#include <algorithm>

#define eps_value 1e-6

struct PlaneType{
    Eigen::Vector3d p[4];
};

/**
 * 在规则立方体网格中筛选与圆锥视场相交的盒子。
 * 先沿最接近视轴的主轴逐层扫描，再用点/线/面相交测试确认边界盒，
 * 用于在昂贵的点云渲染前快速裁剪全局环境。
 */
class FOV_Checker{
public:
    FOV_Checker();
    ~FOV_Checker();
    void Set_Env(BoxPointType env_param);
    void Set_BoxLength(double box_len_param);
    /** 返回深度 depth、半角 theta 的圆锥视场可能覆盖的环境盒。 */
    void check_fov(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, vector<BoxPointType> &boxes);
    bool check_box(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box);
    bool check_line(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d line_p, Eigen::Vector3d line_vec);
    bool check_surface(Eigen::Vector3d cur_pose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane);
    bool check_point(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point);
    bool check_box_in_env(BoxPointType box);    
private:
    BoxPointType env;
    double box_length;
    FILE *fp;

};
