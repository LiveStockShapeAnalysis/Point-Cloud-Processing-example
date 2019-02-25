#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
using namespace pcl::console;
int
main (int argc, char** argv)
{

	if(argc<2)
	{
		std::cout<<".exe xx.pcd -b_n 0 -kn 50 -st_n 30 -ct_n 0.05 -bc 0 -fc 10 -nc 0 -dt 10 -ct 6 -rt 5 -mt 600" <<endl;

		return 0;
	}
	time_t start,end,diff[5],option;
	start = time(0); 
	bool Bool_Cuting=false,b_n=false;
	int MinClusterSize=600,KN_normal=50;
	float far_cuting=10,near_cuting=0,DistanceThreshold=10.0,ColorThreshold=6,RegionColorThreshold=5,SmoothnessThreshold=30.0,CurvatureThreshold=0.05;

	parse_argument (argc, argv, "-b_n", b_n);
	parse_argument (argc, argv, "-kn", KN_normal);
	parse_argument (argc, argv, "-st_n", SmoothnessThreshold);
	parse_argument (argc, argv, "-ct_n", CurvatureThreshold);


	parse_argument (argc, argv, "-bc", Bool_Cuting);
	parse_argument (argc, argv, "-fc", far_cuting);
	parse_argument (argc, argv, "-nc", near_cuting);
	
	parse_argument (argc, argv, "-dt", DistanceThreshold);
	parse_argument (argc, argv, "-ct", ColorThreshold);
	parse_argument (argc, argv, "-rt", RegionColorThreshold);
	parse_argument (argc, argv, "-mt", MinClusterSize);



	//frist step load the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	end = time(0); 
	diff[0] = difftime (end, start); 
	PCL_INFO ("\Loading pcd file takes(seconds): %d\n", diff[0]); 
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

	//Noraml estimation step(1 parameter)
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree1 = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (KN_normal);
	normal_estimator.compute (*normals);
	end = time(0); 
	diff[1] = difftime (end, start)-diff[0]; 
	PCL_INFO ("\Estimating normal takes(seconds): %d\n", diff[1]); 
	//Optional step: cutting away the clutter scene too far away from camera
	pcl::IndicesPtr indices (new std::vector <int>);
	if(Bool_Cuting)//是否通过z轴范围对待处理数据进行裁剪
	{

		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (near_cuting, far_cuting);
		pass.filter (*indices);
	}
	// Region growing RGB 
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud (cloud);
	if(Bool_Cuting)reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (DistanceThreshold);
	reg.setPointColorThreshold (ColorThreshold);
	reg.setRegionColorThreshold (RegionColorThreshold);
	reg.setMinClusterSize (MinClusterSize);
	if(b_n)
	{
		reg.setSmoothModeFlag(true);
		reg.setCurvatureTestFlag(true);

		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (SmoothnessThreshold / 180.0 * M_PI);
		reg.setCurvatureThreshold (CurvatureThreshold);
	}
	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);
	end = time(0); 
	diff[2] = difftime (end, start)-diff[0]-diff[1]; 
	PCL_INFO ("\RGB region growing takes(seconds): %d\n", diff[2]); 

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("点云库PCL学习教程第二版实例-基于颜色的区域生长算法实现点云分割");
	viewer.showCloud (colored_cloud);
	while (!viewer.wasStopped ())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}

	return (0);
}