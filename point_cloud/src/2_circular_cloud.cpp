#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(){

    pcl::PointCloud<pcl::PointXYZRGB> cloud; // Datatype used in pcl library to represent a  3d pint cloud. Templeted class (<pcl::PointXYZ>) represents the type of cloud being represented
    // In this case a dense point cloud is created
    
    double radius = 3.0;
    int num_points = 50;
    double angular_step = (2 * M_PI)/num_points;
    
    for (int i=0; i<=num_points; i++)
    {
        pcl::PointXYZRGB point; // Create a single point of the point cloud and define its coordinates and colour
        double angle = i * angular_step;
        point.x = radius * std::cos(angle);
        point.y = radius * std::sin(angle);
        point.z = 1.0;

        point.r = 255 * std::cos(angle);
        point.g = 255 * std::sin(angle);
        point.b = 255 * std::cos(angle + M_PI_2);

        cloud.push_back(point); // The above class creates a vector bcz of the PointXYZ as in 3d. Adding a single point to the cloud
}
    std::string path = "/home/adinisal/ros2_ws/src/point_cloud/circular_cloud.pcd";

    pcl::io::savePCDFileASCII(path, cloud);

    std::cout<<cloud.size(); // Tells how many points the cloud contains

    return 0;

}
