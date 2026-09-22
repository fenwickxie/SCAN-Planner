#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/** 返回标量的符号，用于确定射线在每个栅格轴上的步进方向。 */
double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

/**
 * 使用 Amanatides-Woo 体素遍历算法列出线段经过的整数栅格。
 * start/end 使用栅格坐标而非米；输出仅保留半开边界 [min, max) 内的体素。
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * 可迭代版本的体素射线遍历器。
 * GridMap 用 step() 流式消费体素，避免为每条传感器射线分配 vector。
 */
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  /** 初始化射线；起终点落入同一体素时返回 false。 */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /** 输出当前体素并前进；到达终点体素后返回 false。 */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_