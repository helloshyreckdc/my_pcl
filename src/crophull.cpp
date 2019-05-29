#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/filters/crop_hull.h>

void auto_area(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,double p1,double p01,double p02,double b1,double b2,pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
for(int i = 0; i< input_cloud->points.size(); i++)
{
input_cloud->points[i].z = 0;
}
	std::vector<pcl::Vertices> vertices;
	pcl::Vertices vt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud(new pcl::PointCloud<pcl::PointXYZ>);
	segmentCloud->push_back(pcl::PointXYZ(0,b1,0));
//	segmentCloud->push_back(pcl::PointXYZ(0,b2,0));
	segmentCloud->push_back(pcl::PointXYZ(p1,p02,0));
	segmentCloud->push_back(pcl::PointXYZ(p1,p01,0));
	segmentCloud->push_back(pcl::PointXYZ(0,b1,0));
	vt.vertices.push_back(1);
	vt.vertices.push_back(2);
	vt.vertices.push_back(3);
	vt.vertices.push_back(4);
//	vt.vertices.push_back(5);
	vertices.push_back(vt);
	pcl::CropHull<pcl::PointXYZ> cropHull;
	cropHull.setInputCloud(input_cloud);
	cropHull.setHullIndices(vertices);
	cropHull.setHullCloud(segmentCloud);
	cropHull.setDim(2); 
	cropHull.setCropOutside(true);
	cropHull.filter(*output_cloud);
}
	int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
//	reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
	reader.read<pcl::PointXYZ> ("/home/shyreckdc/catkin_ws/src/my_pcl/pcd_materials/cokecan.pcd", *cloud);

	// Visualization
//	pcl::visualization::CloudViewer viewer("点云滤波之前");   
//    viewer.showCloud(cloud);  

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
//	sor.filter (*cloud_filtered);
	auto_area(cloud, 100,0,100,0,100,cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	//visualization
	pcl::visualization::CloudViewer viewer1("点云滤波之后");   
int count = 0;
while(count<1000000)
{
    viewer1.showCloud(cloud_filtered);  
    ++count;
}
	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ> ("cokecan_from_scene_inlier.pcd", *cloud_filtered, false);

//	sor.setNegative (true);
//	sor.filter (*cloud_filtered);
//	writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	return (0);
}
