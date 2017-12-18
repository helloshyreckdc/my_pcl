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
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_final (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2::Ptr cloud_downsample (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::ExtractIndices<pcl::PointXYZ> extract;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
std::string str_in;

Eigen::Matrix4f Tm = Eigen::Matrix4f::Identity();
int count_icp = 1;
//std::string str_out;
    void 
parseCommandLine(int argc, char *argv[])
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

void handlePointCallback(const pcl::PCLPointCloud2ConstPtr& cloud)
{
    if(count_icp == 1)
    {


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_in);

	// downsample
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.002f, 0.002f, 0.002f);
	sor.filter (*cloud_downsample);

	//	pcl::PCLPointCloud2::Ptr cloud2(*cloud);
	pcl::fromPCLPointCloud2(*cloud_downsample, *cloud_out);

	// passthrough filter
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_out);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.1, 0.1);
	/*	pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.2, 0.2);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.4, 0.8);   */
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_out_filtered);
	pcl::io::savePCDFileASCII ("test_scene.pcd", *cloud_out_filtered);


	// planar segmentation 
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);


	int i = 0, nr_points = (int) cloud_out_filtered->points.size ();
	// While 30% of the original cloud is still there
	while (cloud_out_filtered->points.size () > 0.3 * nr_points)
	{
	    // Segment the largest planar component from the remaining cloud
	    seg.setInputCloud (cloud_out_filtered);
	    seg.segment (*inliers, *coefficients);
	    if (inliers->indices.size () == 0)
	    {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		break;
	    }

	    // Extract the inliers
	    extract.setInputCloud (cloud_out_filtered);
	    extract.setIndices (inliers);
	    extract.setNegative (true);
	    extract.filter (*cloud_segmented);
	    std::cerr << "PointCloud representing the box segmentation: " << cloud_segmented->width * cloud_segmented->height << " data points." << std::endl;
	    std::cerr << "PointCloud representing the cloud_filtered: " << cloud_out_filtered->width * cloud_out_filtered->height << " data points." << std::endl;

	    std::stringstream ss;
	    ss << "./cube5cm2_segmented_" << i << ".pcd";
	    //  writer.write<pcl::PointXYZ> (ss.str (), *cloud_segmented, false);

	    // Create the filtering object
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	    cloud_out_filtered.swap (cloud_f);
	    i++;
	}

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
	sor2.setInputCloud (cloud_segmented);
	sor2.setMeanK (30);
	sor2.setStddevMulThresh (1);
	sor2.filter (*cloud_filtered);

	pcl::io::savePCDFileASCII ("final.pcd", *cloud_filtered);
	/*	seg.setInputCloud (cloud_out_filtered);
		seg.segment (*inliers, *coefficients);

	// Extract the inliers
	extract.setInputCloud (cloud_out_filtered);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud_out_final);
	pcl::io::savePCDFileASCII ("test_scene_without_planar.pcd", *cloud_out_final);
	 */
	
	icp.setMaximumIterations (50);
	icp.setInputSource (cloud_filtered);
	icp.setInputTarget(cloud_in);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	    icp.getFitnessScore() << std::endl;
	Tm = icp.getFinalTransformation();
	ROS_INFO("Succeed!");

    }
    count_icp ++;
    //	Tm (0,0) = 0.985392;
    //	Tm (0,1) = -0.156762; 
    //	Tm (0,2) = -0.066548;
    //	Tm (0,3) = -0.0120812;
    //	Tm (1,0) = 0.140639;
    //	Tm (1,1) = 0.528688;
    //	Tm (1,2) = 0.837085;
    //	Tm (1,3) = -0.106509;
    //	Tm (2,0) = -0.0960403;
    //	Tm (2,1) = -0.834216;
    //	Tm (2,2) = 0.543011;
    //	Tm (2,3) = 1.33569;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_target_tf");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("camera/depth/points", 1000, handlePointCallback);

    parseCommandLine (argc, argv);

    if (pcl::io::loadPCDFile (str_in, *cloud_in) < 0)
    {
	std::cout << "Error loading input cloud" << std::endl;
	return (-1);
    }

    ros::Rate loop_rate(1);

    tf::TransformBroadcaster br;



    while(ros::ok())
    {



	tf::Vector3 origin;
	origin.setValue(Tm(0,3),Tm(1,3),Tm(2,3));

	tf::Matrix3x3 tf3d;
	tf3d.setValue(Tm(0,0), Tm(0,1), Tm(0,2), 
		Tm(1,0), Tm(1,1), Tm(1,2), 
		Tm(2,0), Tm(2,1), Tm(2,2));


	std::cout << Tm  << std::endl;
	pcl::transformPointCloud (*cloud_in, *transformed_cloud, Tm);

	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	tf::Transform transform;
	transform.setOrigin(origin);
	transform.setRotation(tfqt);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "cokecan"));

	ros::spinOnce();

	loop_rate.sleep();
    }
    return 0;

}
