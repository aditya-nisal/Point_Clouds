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
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

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


    ////////// VOXEL DOWNSAMPLING

    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with voxel filter applied
    // Voxel Grid
    pcl::VoxelGrid<PointT> voxel_filter; // Creates an instance of voxel grid downsampling
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05); //Has the x, y, z length of voxel (here: 0.05*0.05*0.05 cube formed)
    voxel_filter.filter(*voxel_cloud); // Downsampling
    

    ////////// PASS THROUGH FILTER   

    pcl::PointCloud<PointT>::Ptr pass_through_cloud (new pcl::PointCloud<PointT>); // Similarly creating a cloud of same type for the output cloud with pass through filter applied
    
        // Passthrough filter along x
    pcl::PassThrough<PointT> passing_x; // Creating an instance of passthrough filter class
    passing_x.setInputCloud(voxel_cloud); 
    passing_x.setFilterFieldName("x"); // Filtering should be done based on x coordinate of points
    passing_x.setFilterLimits(-1.4, 1.4); // Range of filtering. (Only points with x coordinate in range -1.4 - 1.4 will be retained)
    passing_x.filter(*pass_through_cloud); // Filtering

    // Passthrough filter along y
    pcl::PassThrough<PointT> passing_y; // Creating an instance of passthrough filter class
    passing_y.setInputCloud(pass_through_cloud); 
    passing_y.setFilterFieldName("y"); // Filtering should be done based on y coordinate of points
    passing_y.setFilterLimits(-1.4, 1.4); // Range of filtering. (Only points with y coordinate in range -1.4 - 1.4 will be retained)
    passing_y.filter(*pass_through_cloud); // Filtering


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
    indices_extractor.setNegative(true); // Do we need the negative of what we segmented (Set true to get the cylinders)
    indices_extractor.filter(*plane_segmented_cloud); // Adding the indices(inlier) points to the output cloud


    ////////// CYLINDER SEGMENTATION

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); // Creates a shared ptr to a new pcd that holds normal
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>()); // Shared poniter to a new kdtree is created
    pcl::PointCloud<PointT>::Ptr cylinder_cloud (new pcl::PointCloud<PointT>()); // Similarly creating a cloud of same type for the output cloud for the segmented cylinder

    pcl::NormalEstimation<PointT, pcl::Normal> normals_estimator; // Instanc eof normal estimator class. Will be used to estimate normals
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor; // Instance of class . will be used to segment geometries
    pcl::ExtractIndices<PointT> cylinder_indices_extractor; // Instance of class. Will be used to extract the inlier points

    // Configure and run the normal estimation
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(plane_segmented_cloud);
    normals_estimator.setKSearch(30);
    normals_estimator.compute(*cloud_normals);

    // Configured and run the cylinder segmentation
    cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentor.setDistanceThreshold(0.05);

    cylinder_segmentor.setInputCloud(plane_segmented_cloud);
    cylinder_segmentor.setInputNormals(cloud_normals);
    cylinder_segmentor.segment(*inliers, *coefficients);

    // Previously computed inliers are used to extract the points of the cylinder
    cylinder_indices_extractor.setInputCloud(plane_segmented_cloud);
    cylinder_indices_extractor.setIndices(inliers);
    cylinder_indices_extractor.setNegative(false);
    cylinder_indices_extractor.filter(*cylinder_cloud);


    //////////  WRITING THE CLOUD

    CloudSaver("cylinder_cloud.pcd", path, cylinder_cloud);
    return 0;

}
