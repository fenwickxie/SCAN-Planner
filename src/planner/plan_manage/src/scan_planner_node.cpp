#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/scan_replan_fsm.h>

using namespace scan_planner;

int main(int argc, char **argv)
{
  // 主节点本身只负责组装 FSM 并进入 ROS 事件循环；地图、规划和安全检查
  // 都由 SCANReplanFSM::init() 创建的订阅器与定时器驱动。
  ros::init(argc, argv, "scan_planner_node");
  ros::NodeHandle nh("~");

  SCANReplanFSM scan_replan;

  scan_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
