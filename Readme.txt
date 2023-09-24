POINT CLOUD:

ctrl + shift + "+" Will make the points larger



COMMAND LINE TOOLS:

pcl_viewer <relative path to the .pcd file>
pcl_viewer --multiview <number of files to open (eg. n)> <relative path to file1> <relative path to file 2> ...... <relative path to file n>

RUNNING THE PROJECT

# Launches the pointcloud in rviz
ros2 launch point_cloud process_kitti.launch.py      