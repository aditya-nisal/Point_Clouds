#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

using namespace std::chrono_literals;


class VoxelGridFilter : public rclcpp::Node
{
  public:
    VoxelGridFilter()
    : Node("minimal_publisher"), count_(0)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/kitti/point_cloud", 10, std::bind(&VoxelGridFilter::timer_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_cloud", 10);

    }

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Convert and store ros2 mssage type into point cloud DS
      pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Applying Voxel Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>); // Similarly creating a cloud of same type for the output cloud with voxel filter applied
    // Voxel Grid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter; // Creates an instance of voxel grid downsampling
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.1, 0.1, 0.1); //Has the x, y, z length of voxel (here: 0.05*0.05*0.05 cube formed)
    voxel_filter.filter(*voxel_cloud); // Downsampling

    // Convert to ROS2 message again
    sensor_msgs::msg::PointCloud2 voxel_cloud_ros2;
      pcl::toROSMsg(*voxel_cloud, voxel_cloud_ros2);
      voxel_cloud_ros2.header.frame_id = "map";
      voxel_cloud_ros2.header.stamp = this->now();

        std::cout << "PointCLoud size before voxelization: " << pcl_cloud->size() <<std::endl;
        std::cout << "PointCLoud size after voxelization: " << voxel_cloud->size() <<std::endl;

      publisher_->publish(voxel_cloud_ros2);
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;    
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridFilter>());
  rclcpp::shutdown();
  return 0;
}


