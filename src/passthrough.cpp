#include <eigen3/Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

std::string str_in;
pcl::ExtractIndices<pcl::PointXYZ> extract;

void parseCommandLine(int argc, char *argv[])
{
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 1)
	{
		std::cout << "Filenames missing.\n";
		exit(-1);
	}
	str_in = argv[filenames[0]];
	//	str_out = argv[filenames[1]];
}

	int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_final (new pcl::PointCloud<pcl::PointXYZ>);

	parseCommandLine (argc, argv);

	if (pcl::io::loadPCDFile (str_in, *cloud) < 0)
	{
		std::cout << "Error loading input cloud" << std::endl;
		return (-1);
	}
	// Fill in the cloud data
	//	cloud->width  = 5;
	//	cloud->height = 1;
	//	cloud->points.resize (cloud->width * cloud->height);
	//
	//	for (size_t i = 0; i < cloud->points.size (); ++i)
	//	{
	//		cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	//		cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	//		cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	//	}
	//
	//	std::cerr << "Cloud before filtering: " << std::endl;
	//	for (size_t i = 0; i < cloud->points.size (); ++i)
	//		std::cerr << "    " << cloud->points[i].x << " " 
	//			<< cloud->points[i].y << " " 
	//			<< cloud->points[i].z << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (atof(argv[2]), atof(argv[3]));
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (atof(argv[4]), atof(argv[5]));
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_z);

	pass.setInputCloud (cloud_filtered_z);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (atof(argv[6]), atof(argv[7]));
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_x);
//	// planar segmentation 
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//	// Create the segmentation object
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional
//	seg.setOptimizeCoefficients (true);
//	// Mandatory
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setMaxIterations (1000);
//	seg.setDistanceThreshold (0.01);
//	seg.setInputCloud (cloud_filtered_z);
//	seg.segment (*inliers, *coefficients);
//
//	// Extract the inliers
//	extract.setInputCloud (cloud_filtered_z);
//	extract.setIndices (inliers);
//	extract.setNegative (true);
//	extract.filter (*cloud_out_final);
//	pcl::io::savePCDFileASCII ("test_scene_without_planar.pcd", *cloud_out_final);
//	//	std::cerr << "Cloud after filtering: " << std::endl;
//	//	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
//	//		std::cerr << "    " << cloud_filtered->points[i].x << " " 
//	//			<< cloud_filtered->points[i].y << " " 
//	//			<< cloud_filtered->points[i].z << std::endl;

	pcl::io::savePCDFileASCII ("passthrough_test_scene.pcd", *cloud_filtered_x);
	return (0);
}
