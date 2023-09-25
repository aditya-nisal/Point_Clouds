#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>


using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

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
      pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
    // Convert and store ros2 mssage type into point cloud DS
      pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Applying Voxel Filter
    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with voxel filter applied
    // Voxel Grid
    pcl::VoxelGrid<PointT> voxel_filter; // Creates an instance of voxel grid downsampling
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.1, 0.1, 0.1); //Has the x, y, z length of voxel (here: 0.05*0.05*0.05 cube formed)
    voxel_filter.filter(*voxel_cloud); // Downsampling


    // Road Segmentation
    /*normal extractor->uses tree->get road normals->segment road from normals->get road inliers->get road coefficients->extract road idices->store into road cloud*/
    pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr road_normals (new pcl::PointCloud<pcl::Normal>);
 
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_seg_from_normals;
    pcl::PointIndices::Ptr road_inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr road_coefficients (new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> road_extract_indices;
    pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);


    // Normal Extraction

    normal_extractor.setSearchMethod(tree); // Sets searrch method of normal estimator to be kdtree
    normal_extractor.setInputCloud(voxel_cloud); // Sets input cloud
    normal_extractor.setKSearch(30); // Neighbouring points to be used for normal estimation
    normal_extractor.compute(*road_normals); // Compute normals at each point and store them

    // Configured and run the cylinder segmentation
    road_seg_from_normals.setOptimizeCoefficients(true);
    road_seg_from_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE); // Model type to cylinder
    road_seg_from_normals.setMethodType(pcl::SAC_RANSAC);
	road_seg_from_normals.setNormalDistanceWeight(0.5);
	road_seg_from_normals.setMaxIterations(100);
	road_seg_from_normals.setDistanceThreshold(0.4);
    road_seg_from_normals.setInputCloud(voxel_cloud);
    road_seg_from_normals.setInputNormals(road_normals);
    road_seg_from_normals.segment(*road_inliers, *road_coefficients);

    // Extracting Road cloud 
    road_extract_indices.setInputCloud(voxel_cloud);
    road_extract_indices.setIndices(road_inliers);
    road_extract_indices.setNegative(false);
    road_extract_indices.filter(*road_cloud);

    // Extracting the clusters
    /* single segmented cluster->all clusters at a given time extraacted with our segmentation->arranging cluster based on indices->cluster extracted based on 
    eucludian distance*/
    pcl::PointCloud<PointT>::Ptr single_segmented_cluster (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> eucludian_cluster_extractor;

    // Eucludian cluster
    tree->setInputCloud(road_cloud); // Giving the input to the tree
    eucludian_cluster_extractor.setMinClusterSize(100);
    eucludian_cluster_extractor.setMaxClusterSize(2000);
    eucludian_cluster_extractor.setSearchMethod(tree);
    eucludian_cluster_extractor.setInputCloud(road_cloud);
    eucludian_cluster_extractor.extract(); // Extract all the clusters in the road cloud given that these are the properties




    // Convert to ROS2 message again
    sensor_msgs::msg::PointCloud2 traffic_seg_ros2;
      pcl::toROSMsg(*road_cloud, traffic_seg_ros2);
      traffic_seg_ros2.header.frame_id = "map";
      traffic_seg_ros2.header.stamp = this->now();

        // std::cout << "PointCLoud size before voxelization: " << pcl_cloud->size() <<std::endl;
        // std::cout << "PointCLoud size after voxelization: " << voxel_cloud->size() <<std::endl;

      publisher_->publish(traffic_seg_ros2);
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


