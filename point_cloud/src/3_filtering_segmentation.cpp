#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT; // Flexible tempelate to hold the data into a flexible pcl point

int main(){

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); // SHared pointer takes of the autonmatic memory allocation to the extent it is required
                                                                      // Creates a sharedPtr of the point data type
    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with voxel filter applied
    pcl::PointCloud<PointT>::Ptr pass_through_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with pass through filter applied
    
    // Declaring cloud reader and cloud write to read input cloud and write the out cloud
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;
    
    // Reading the cloud from the path and storing it
    std::string path =  "/home/adinisal/ros2_ws/src/point_cloud/";
    std::string cloud_name =  "tb3_world.pcd";
    std::string output_name =  "pass_xy_cloud.pcd";
    cloud_reader.read(path+cloud_name, *cloud); // Reading cloud fromthe path and storing it in the empty cloud created

    // Voxel Grid
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);

    // Passthrough filter along x
    pcl::PassThrough<PointT> passing_x;
    passing_x.setInputCloud(voxel_cloud);
    passing_x.setFilterFieldName("x");
    passing_x.setFilterLimits(-1.4, 1.4);
    passing_x.filter(*pass_through_cloud);

    // Passthrough filter along y
    pcl::PassThrough<PointT> passing_y;
    passing_y.setInputCloud(pass_through_cloud);
    passing_y.setFilterFieldName("y");
    passing_y.setFilterLimits(-1.4, 1.4);
    passing_y.filter(*pass_through_cloud);

    // Cloud Writing
    cloud_writer.write<PointT>(path+output_name, *pass_through_cloud);

    return 0;

}
