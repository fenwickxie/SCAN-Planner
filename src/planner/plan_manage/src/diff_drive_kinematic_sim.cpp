#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
ros::Publisher odom_pub;
ros::Subscriber cmd_sub;
ros::Timer sim_timer;
tf::TransformBroadcaster *tf_broadcaster = nullptr;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double yaw = 0.0;

double v_cmd = 0.0;
double w_cmd = 0.0;
double v_world_x = 0.0;
double v_world_y = 0.0;

double max_v = 0.4;
double max_w = 0.8;
double cmd_timeout = 0.3;
double sim_rate = 100.0;
bool publish_tf = false;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";

ros::Time last_cmd_time;
ros::Time last_sim_time;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void loadParamWithFallback(const ros::NodeHandle &nh, const std::string &private_name,
                           const std::string &fallback_name, double &value, double default_value)
{
  if (nh.getParam(private_name, value))
    return;
  if (ros::param::get(fallback_name, value))
    return;
  value = default_value;
}

void cmdCallback(const geometry_msgs::TwistConstPtr &msg)
{
  v_cmd = clamp(msg->linear.x, -max_v, max_v);
  w_cmd = clamp(msg->angular.z, -max_w, max_w);
  last_cmd_time = ros::Time::now();
}

void publishOdom(const ros::Time &stamp)
{
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = v_world_x;
  odom.twist.twist.linear.y = v_world_y;
  odom.twist.twist.angular.z = w_cmd;
  odom_pub.publish(odom);

  if (!publish_tf || tf_broadcaster == nullptr)
    return;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

void simCallback(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  double dt = (now - last_sim_time).toSec();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  double v = v_cmd;
  double w = w_cmd;
  if ((now - last_cmd_time).toSec() > cmd_timeout)
  {
    v = 0.0;
    w = 0.0;
  }

  v_world_x = v * std::cos(yaw);
  v_world_y = v * std::sin(yaw);
  x += v_world_x * dt;
  y += v_world_y * dt;
  yaw = normalizeAngle(yaw + w * dt);

  publishOdom(now);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_drive_kinematic_sim");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::param::param("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ros::param::param("/init_x", x, 0.0);
  ros::param::param("/init_y", y, 0.0);
  ros::param::param("/init_z", z, 0.3);
  nh.param("init_yaw", yaw, 0.0);
  loadParamWithFallback(nh, "max_v", "/diff_drive_controller/max_v", max_v, 0.4);
  loadParamWithFallback(nh, "max_w", "/diff_drive_controller/max_w", max_w, 0.8);
  nh.param("cmd_timeout", cmd_timeout, 0.3);
  nh.param("sim_rate", sim_rate, 100.0);
  nh.param("publish_tf", publish_tf, false);
  nh.param("frame_id", frame_id, std::string("world"));
  nh.param("child_frame_id", child_frame_id, std::string("base"));

  tf::TransformBroadcaster broadcaster;
  tf_broadcaster = &broadcaster;

  odom_pub = node.advertise<nav_msgs::Odometry>(body_pose_topic, 100);
  cmd_sub = node.subscribe("cmd_vel", 20, cmdCallback, ros::TransportHints().tcpNoDelay());

  last_cmd_time = ros::Time::now();
  last_sim_time = ros::Time::now();
  sim_timer = node.createTimer(ros::Duration(1.0 / sim_rate), simCallback);

  ROS_WARN("[diff_drive_kinematic_sim] ready.");

  ros::spin();
  return 0;
}
