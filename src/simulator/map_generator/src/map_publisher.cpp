#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

// 将离线 PCD 作为静态全局环境周期发布。该点云供 local_sensing 渲染器
// 使用，而不是直接作为规划地图；规划器仍只融合模拟传感器的局部观测。
int main(int argc, char** argv) {
  ros::init(argc, argv, "map_pub");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  std::string file_name;
  if (argc > 1) {
    file_name = argv[1];
  }
  private_node.param("file_name", file_name, file_name);

  std::string frame_id;
  std::string cloud_topic;
  double publish_rate;
  double downsample_res;
  double map_offset_x;
  double map_offset_y;
  double map_offset_z;
  private_node.param("frame_id", frame_id, std::string("world"));
  private_node.param("cloud_topic", cloud_topic, std::string("/map_generator/global_cloud"));
  private_node.param("publish_rate", publish_rate, 3.0);
  private_node.param("downsample_res", downsample_res, 0.0);
  private_node.param("map_offset_x", map_offset_x, 0.0);
  private_node.param("map_offset_y", map_offset_y, 0.0);
  private_node.param("map_offset_z", map_offset_z, 0.0);

  if (file_name.empty()) {
    ROS_ERROR("[map_pub] No PCD file specified. Pass it as an arg or set ~file_name.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) < 0) {
    ROS_ERROR_STREAM("[map_pub] Failed to read PCD file: " << file_name);
    return 1;
  }

  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, finite_indices);

  // 可选体素滤波降低渲染器负载；叶尺寸过大会吞掉细杆和薄墙。
  if (downsample_res > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setInputCloud(cloud.makeShared());
    voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
    voxel_sampler.filter(cloud);
  }

  if (map_offset_x != 0.0 || map_offset_y != 0.0 || map_offset_z != 0.0) {
    for (auto& point : cloud.points) {
      point.x += map_offset_x;
      point.y += map_offset_y;
      point.z += map_offset_z;
    }
  }

  if (cloud.empty()) {
    ROS_ERROR_STREAM("[map_pub] PCD file has no valid XYZ points: " << file_name);
    return 1;
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;

  // latch=true 使后启动的渲染节点无需等待下一周期即可收到最后一张地图。
  ros::Publisher cloud_pub =
      node.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10, true);

  ROS_INFO_STREAM("[map_pub] Loaded " << cloud.points.size() << " points from " << file_name
                                      << ", publishing " << cloud_topic << " in frame "
                                      << frame_id);

  ros::Rate rate(std::max(0.1, publish_rate));
  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    cloud_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
