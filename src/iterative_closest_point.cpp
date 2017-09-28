#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
std::string str_in;
std::string str_out;
	void 
parseCommandLine(int argc, char *argv[])
{
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 2)
	{
		std::cout << "Filenames missing.\n";
		exit(-1);
	}
	str_in = argv[filenames[0]];
	str_out = argv[filenames[1]];
}
	int
main (int argc, char** argv)
{

	parseCommandLine (argc, argv);

	if (pcl::io::loadPCDFile (str_in, *cloud_in) < 0)
	{
		std::cout << "Error loading input cloud" << std::endl;
		return (-1);
	}

	if (pcl::io::loadPCDFile (str_out, *cloud_out) < 0)
	{
		std::cout << "Error loading output cloud" << std::endl;
		return (-1);
	}
	// Fill in the CloudIn data
	//	cloud_in->width    = 200;
	//	cloud_in->height   = 1;
	//	cloud_in->is_dense = false;
	//	cloud_in->points.resize (cloud_in->width * cloud_in->height);
	//	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	//	{
	//		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	//		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	//		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	//	}
	//	std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
	//		<< std::endl;
	//	for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
	//		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
	//			cloud_in->points[i].z << std::endl;
	//	*cloud_out = *cloud_in;
	//	std::cout << "size:" << cloud_out->points.size() << std::endl;
	//	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	//		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	//	std::cout << "Transformed " << cloud_in->points.size () << " data points:"
	//		<< std::endl;
	//	for (size_t i = 0; i < cloud_out->points.size (); ++i)
	//		std::cout << "    " << cloud_out->points[i].x << " " <<
	//			cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;



	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;


	return (0);
}
