#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(){

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2); // Reading into a pointer. It will create a shared pointer also allocating a memory for the cloud

    pcl::PCDReader cloud_reader; // Calling the class of pcd reader theat will be utilized to read the point cloud

    std::string  path = "/home/adinisal/ros2_ws/src/point_cloud/table_scene.pcd";

    cloud_reader.read(path, *cloud); // Reading the cloud by putting the cloud into the cloud pointer we created 

    std::cout<<"Number of points: "<<cloud->width * cloud->height <<std::endl; // We print out a 2d area of the point cloud

    return 0;
}