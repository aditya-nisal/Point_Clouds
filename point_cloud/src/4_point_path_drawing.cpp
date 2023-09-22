#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <filesystem>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


// JUST CREATING A VERY BASIC AND SIMPLE PATH BETWEEN THE ENDS OF HE DETECTED PLANE

int main() {
    std::string pcd_file_name = "plane_seg.pcd";
    std::filesystem::path ros2_ws_path = std::filesystem::current_path();
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/point_cloud/";
    std::string pcd_file_path = point_cloud_dir/pcd_file_name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud);

    pcl::visualization::PCLVisualizer viewer("Point cloud with path");
    viewer.addPointCloud(cloud, "Ground_plane");

    float min_x, min_y, max_y = 0;
    for (int i = 0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZ point = cloud->points[i];
        if (point.x < min_x){
            min_x = point.x;
        }
        if (point.y < min_y){
            min_y = point.y;
        }
        if (point.y > max_y){
            max_y = point.y;
        }
    }

    float ball_radius = 0.15;
    float ball_x = min_x;
    float ball_y = min_y;

    int i = 0;
    while (ball_y <= max_y){
   
    pcl::PointXYZ point;
    point.x = ball_x;
    point.y = ball_y;

    viewer.addSphere(point, ball_radius, 1.0, 0.0, 0.0, "Sphere"+std::to_string(i++));
    ball_y += ball_radius * 1.5;
    }
    viewer.spin();

    return 0;
}