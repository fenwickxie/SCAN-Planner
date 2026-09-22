#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace scan_planner
{
  /**
   * 任意维度 B 样条表示。
   * 本项目的位置轨迹使用 3xN 控制点矩阵：每列是一个三维控制点。
   * 类名保留历史命名，但 setKnot() 也允许控制器重建非均匀节点向量。
   */
  class UniformBspline
  {
  private:
    // 行数是空间维度，列数是控制点个数；三维轨迹通常为 3xN。
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p: 次数；n+1: 控制点数；m+1: 节点数
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, feasibility_tolerance_; // physical limits and feasibility tolerance

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    /** 使用数值稳定的 De Boor 递推在节点参数 u 处求值。 */
    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    /**
     * 从 K 个采样点及首尾速度/加速度反解三次均匀 B 样条控制点。
     * start_end_derivative 顺序为 start_vel、end_vel、start_acc、end_acc；
     * 输出为 3x(K+2)，以额外控制点满足四个边界导数约束。
     */
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    /** 检查控制点导数充分条件，并给出使速度/加速度恢复限制所需的时间缩放比。 */
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace scan_planner
#endif
