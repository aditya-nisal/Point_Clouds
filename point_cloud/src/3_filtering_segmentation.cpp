#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGB PointT; // Flexible tempelate to hold the data into a flexible pcl point

void CloudSaver(const std::string& file_name, std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<PointT>(path+std::string(file_name), *cloud_arg);
}

int main(){

    ////////// READING THE CLOUD

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); // SHared pointer takes of the autonmatic memory allocation to the extent it is required
                                                                      // Creates a sharedPtr of the point data type. new pcl::... means alllocating memory to the cloud
    pcl::PCDReader cloud_reader;
    std::string path =  "/home/adinisal/ros2_ws/src/point_cloud/";
    std::string input_cloud =  "tb3_world.pcd";
    cloud_reader.read(path+input_cloud, *cloud); // Reading cloud fromthe path and storing it in the empty cloud created


    ////////// VOXEL FILTER

    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with voxel filter applied
    // Voxel Grid
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);
    

    ////////// PASS THROUGH FILTER   

    pcl::PointCloud<PointT>::Ptr pass_through_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with pass through filter applied
    
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


    ////////// PLANER SEGMENTATION 

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Initialize a shared ptr to a new instance of inliers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // Initialize a shared ptr to a new instance of coefficients that fit the equation. Here it is a pplane
    pcl::PointCloud<PointT>::Ptr plane_segmented_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with th plane segmented cloud points
    pcl::SACSegmentation<PointT> plane_segmentor; // SAC class instance that contains model to segment
    pcl::ExtractIndices<PointT> indices_extractor; // Extract segmented points

    plane_segmentor.setInputCloud(pass_through_cloud);
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE); // Specifing the moel to Segment the plane
    plane_segmentor.setMethodType(pcl::SAC_RANSAC); // Method type for the seg- RANSAC
    plane_segmentor.setDistanceThreshold(0.01); // Distance this segmentation will have from the segmented plane and other points
    plane_segmentor.segment(*inliers, *coefficients); //Performing segmentation. Inliers- indices

    indices_extractor.setInputCloud(pass_through_cloud);
    indices_extractor.setIndices(inliers); // Providing the indices for the cloud
    indices_extractor.setNegative(false); // Do we need the negative of what we segmented
    indices_extractor.filter(*plane_segmented_cloud); // Adding the indices(inlier) points to the output cloud


    //////////  WRITING THE CLOUD

    CloudSaver("plane_seg.pcd", path, plane_segmented_cloud);
    return 0;

}
