#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <vector>
std::string pcd_file_in;


int main (int argc, char** argv)
{
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 1)
	{
		std::cout << "Filenames missing.\n";
		exit (-1);
	}

	pcd_file_in = argv[filenames[0]];

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	//	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	//	reader.read (argv[1], cloud); // Remember to download the file first!
	if (pcl::io::loadPCDFile (pcd_file_in, *cloud) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return (-1);
	}
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
		<< " data points (" << pcl::getFieldsList (*cloud) << ").";


	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
//	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
		<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
	pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filtered1);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_filtered1);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.1, 1.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_pass);
	pcl::PCDWriter writer;
//	writer.write ("_downsampled.pcd", *cloud_pass, 
//			Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
	pcl::io::savePCDFile("_downsampled.pcd", *cloud_pass, false);
	return (0);
}
