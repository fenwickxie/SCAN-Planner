#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double lookahead_min;
double lookahead_max;
double lookahead_gain;
double k_yaw;
double k_v;
double max_v;
double min_v;
double max_w;
double max_lat_acc;
double heading_error_threshold;
double finish_dist;
double search_dt;
std::string body_pose_topic;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[diff_drive_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ok &= loadRequiredParam(nh, "lookahead_min", lookahead_min);
  ok &= loadRequiredParam(nh, "lookahead_max", lookahead_max);
  ok &= loadRequiredParam(nh, "lookahead_gain", lookahead_gain);
  ok &= loadRequiredParam(nh, "k_yaw", k_yaw);
  ok &= loadRequiredParam(nh, "k_v", k_v);
  ok &= loadRequiredParam(nh, "max_v", max_v);
  ok &= loadRequiredParam(nh, "min_v", min_v);
  ok &= loadRequiredParam(nh, "max_w", max_w);
  ok &= loadRequiredParam(nh, "max_lat_acc", max_lat_acc);
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  nh.param("search_dt", search_dt, 0.03);

  if (ok && lookahead_max < lookahead_min)
  {
    ROS_ERROR("[diff_drive_controller] lookahead_max must be >= lookahead_min.");
    ok = false;
  }
  if (ok && (max_v < 0.0 || max_w <= 0.0 || max_lat_acc <= 0.0 || search_dt <= 0.0))
  {
    ROS_ERROR("[diff_drive_controller] max_v/max_w/max_lat_acc/search_dt are invalid.");
    ok = false;
  }

  min_v = std::max(0.0, std::min(min_v, max_v));
  return ok;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

void publishStop(double wz = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.linear.y = 0.0;
  cmd.angular.z = clamp(wz, -max_w, max_w);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

bool parseBspline(const scan_planner::BsplineConstPtr &msg, UniformBspline &pos_traj)
{
  if (!msg || msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
  {
    ROS_WARN("[diff_drive_controller] ignore invalid bspline message.");
    return false;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  UniformBspline rebuilt(pos_pts, msg->order, 0.1);
  rebuilt.setKnot(knots);
  pos_traj = rebuilt;
  return true;
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  UniformBspline pos_traj;
  if (!parseBspline(msg, pos_traj))
    return;

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[diff_drive_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

double searchForwardByArcLength(double start_t, double lookahead_dist)
{
  const double start = clamp(start_t, 0.0, traj_duration);
  Eigen::Vector3d prev = traj[0].evaluateDeBoorT(start);
  double accumulated = 0.0;

  for (double t = start + search_dt; t < traj_duration + 1e-6; t += search_dt)
  {
    const double tc = std::min(t, traj_duration);
    Eigen::Vector3d cur = traj[0].evaluateDeBoorT(tc);
    accumulated += (cur.head<2>() - prev.head<2>()).norm();
    if (accumulated >= lookahead_dist || tc >= traj_duration)
      return tc;
    prev = cur;
  }

  return traj_duration;
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  const Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);
  const Eigen::Vector3d final_pos = traj[0].evaluateDeBoorT(traj_duration);
  const double final_dist = (final_pos.head<2>() - odom_pos.head<2>()).norm();

  if (t_eval >= traj_duration - 1e-4 && final_dist < finish_dist)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const double v_ref = std::min(max_v, vel_des.head<2>().norm());
  const double lookahead_dist = clamp(lookahead_min + lookahead_gain * v_ref, lookahead_min, lookahead_max);
  const double lookahead_t = searchForwardByArcLength(t_eval, lookahead_dist);
  const Eigen::Vector3d lookahead_pt = traj[0].evaluateDeBoorT(lookahead_t);

  const Eigen::Vector2d delta = lookahead_pt.head<2>() - odom_pos.head<2>();
  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  const double x_r = c * delta(0) + s * delta(1);
  const double y_r = -s * delta(0) + c * delta(1);
  const double alpha = normalizeAngle(std::atan2(y_r, x_r));
  const double wz_turn = clamp(k_yaw * alpha, -max_w, max_w);

  if (std::abs(alpha) > heading_error_threshold || x_r < 0.0)
  {
    publishExecutionFrozen(true);
    publishStop(wz_turn);
    last_update_time = now;
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  const double ld2 = std::max(x_r * x_r + y_r * y_r, 1e-4);
  const double curvature = 2.0 * y_r / ld2;
  const double curvature_abs = std::abs(curvature);
  const double curve_v_limit = std::sqrt(max_lat_acc / std::max(curvature_abs, 1e-4));

  double v_cmd = clamp(v_ref + k_v * x_r, min_v, max_v);
  v_cmd = std::min(v_cmd, curve_v_limit);
  if (lookahead_t >= traj_duration - 1e-4 && final_dist < lookahead_max)
    v_cmd = std::min(v_cmd, std::max(0.0, final_dist));

  const double wz_cmd = clamp(curvature * v_cmd + k_yaw * alpha, -max_w, max_w);

  geometry_msgs::Twist cmd;
  cmd.linear.x = v_cmd;
  cmd.linear.y = 0.0;
  cmd.angular.z = wz_cmd;
  cmd_vel_pub.publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_drive_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback, ros::TransportHints().tcpNoDelay());
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[diff_drive_controller] ready.");

  ros::spin();
  return 0;
}
