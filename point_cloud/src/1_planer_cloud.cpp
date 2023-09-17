#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(){

    pcl::PointCloud<pcl::PointXYZ> cloud; // Datatype used in pcl library to represent a  3d pint cloud. Templeted class (<pcl::PointXYZ>) represents the type of cloud being represented

    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(7.0, 8.0, 9.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(10.0, 11.0, 12.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(13.0, 14.0, 15.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(16.0, 17.0, 18.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(19.0, 20.0, 21.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(22.0, 23.0, 24.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
    cloud.push_back(pcl::PointXYZ(25.0, 26.0, 27.0)); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud

    std::string path = "/home/adinisal/ros2_ws/src/point_cloud/plane_cloud.pcd";

    pcl::io::savePCDFileASCII(path, cloud);

    std::cout<<cloud.size(); // Tells how many points the cloud contains

    return 0;

}
