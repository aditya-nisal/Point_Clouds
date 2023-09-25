#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <iostream>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    for(int i =0; i<100; i++)
    {
        pcl::PointXYZ p;
        p.x = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
        p.y = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
        p.z = static_cast<float> (rand()) / static_cast<float> (RAND_MAX);
        cloud->push_back(p);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Create a query point
    pcl::PointXYZ searchPoint; // Given point
    searchPoint.x = 0.1f;
    searchPoint.y = 0.1f;
    searchPoint.z = 0.1f;

    int K = 5; // Neighbours we are looking for
    std::vector<int> indices(K); // Indices of neighbours we obtain
    std::vector<float> distances(K); // Dist of search points to neighbours
    kdtree.nearestKSearch(searchPoint, K, indices, distances); // search tree and store the values

    for (int i = 0; i<indices.size(); i++ ){
        std::cout << "indices[" << i << "]: " << indices[i] << " distances[" << i << "]: " << distances[i] << std::endl;
    }


    return 0;
}