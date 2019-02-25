#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
int
	main (int argc, char** argv)
{
	int max_w_s (20);
	float slope (1.0f);
	float initial_d (0.5f);
	float max_d (3.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),cloud_ground (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground (new pcl::PointIndices);
	if(argc<2)
	{
		std::cout << "Usage: " << argv[0] << " filename.pcd [Options]" << std::endl << std::endl;
		std::cout << "Options:" << std::endl;
		std::cout << "     -mw(20):                     Max window size." << std::endl;
		std::cout << "     -s(1.0):                     Slope." << std::endl;
		std::cout << "     -id(0.5):                    initial distance." << std::endl;
		std::cout << "     -md(3.0):                     Max distance" << std::endl;
		exit(1);
	}
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ> (argv[1], *cloud);
	pcl::console::parse_argument (argc, argv, "-mw", max_w_s);
	pcl::console::parse_argument (argc, argv, "-s", slope);
	pcl::console::parse_argument (argc, argv, "-id", initial_d);
	pcl::console::parse_argument (argc, argv, "-md", max_d);
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// Create the filtering object
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud (cloud);
	pmf.setMaxWindowSize (max_w_s);
	pmf.setSlope (slope);
	pmf.setInitialDistance (initial_d);
	pmf.setMaxDistance (max_d);
	pmf.extract (ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (ground);
	extract.filter (*cloud_ground);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_ground << std::endl;
	int v1,v2;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("点云库PCL学习教程第二版-激光雷达点云地面提取"));

	viewer->createViewPort(0,0,0.5,1,v1);
	viewer->createViewPort(0.5,0,1,1,v2);
	viewer->setBackgroundColor(255,255,255,v1);
	viewer->setBackgroundColor(255,255,255,v2);
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_ground, false);
	viewer->addPointCloud(cloud_ground,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_ground,0,255,0),"cloud_ground",v1);
	// Extract non-ground returns
	extract.setNegative (true);
	extract.filter (*cloud_filtered);
	viewer->addPointCloud(cloud_filtered,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered,0,0,255),"cloud_filtered",v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,"cloud_ground",v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,"cloud_filtered",v2);
	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	viewer->spin();
	writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

	return (0);
}