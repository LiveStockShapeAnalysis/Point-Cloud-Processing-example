#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <iostream>
#include <ostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;
typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;
using namespace pcl::console;
//设置条件函数1
bool
	enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 5.0f)
		return (true);
	else
		return (false);
}
//设置条件函数2
bool
	enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 5.0f)
		return (true);
	if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
		return (true);
	return (false);
}
//设置条件函数3
bool
	customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	if (squared_distance < 10000)
	{
		if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 8.0f)
			return (true);
		if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
			return (true);
	}
	else
	{
		if (fabs ((float)point_a.intensity - (float)point_b.intensity) < 3.0f)
			return (true);
	}
	return (false);
}

int
	main (int argc, char** argv)
{

	if(argc<2)
	{
		std::cout<<".exe xx.pcd -l 40 -r 300.0 -v 1 -m 1/2/3"<<std::endl;

		return 0;
	}//如果输入参数小于1个，输出提示
	bool Visual=true;
	//设置默认输入参数
	float Leaf=40,Radius=300;
	int Method=1;
	//设置输入参数方式
	parse_argument (argc, argv, "-l", Leaf);
	parse_argument (argc, argv, "-r", Radius);
	parse_argument (argc, argv, "-v", Visual);
	parse_argument (argc, argv, "-m", Method);
	// Data containers used
	pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);//创建PointCloud <pcl::PointXYZI>共享指针并进行实例化
	pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);//创建PointCloud <pcl::PointXYZINormal>共享指针并进行实例化
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
	pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
	pcl::console::TicToc tt;

	// Load the input point cloud
	std::cerr << "Loading...\n", tt.tic ();
	pcl::io::loadPCDFile (argv[1], *cloud_in);//打开输入点云文件
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";

	// Downsample the cloud using a Voxel Grid class
	std::cerr << "Downsampling...\n", tt.tic ();
	pcl::VoxelGrid<PointTypeIO> vg;//设置滤波对象
	vg.setInputCloud (cloud_in);//设置需要过滤的点云给滤波对象
	vg.setLeafSize (Leaf,Leaf,Leaf);//设置滤波时创建的栅格边长
	vg.setDownsampleAllData (true);//设置所有的数值域需要被下采样
	vg.filter (*cloud_out);//执行滤波处理，存储输出cloud_out
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";

	// Set up a Normal Estimation class and merge data in cloud_with_normals
	std::cerr << "Computing normals...\n", tt.tic ();
	pcl::copyPointCloud (*cloud_out, *cloud_with_normals);//复制点云
	pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;//创建法线估计对象
	ne.setInputCloud (cloud_out);//设置法线估计对象输入点集
	ne.setSearchMethod (search_tree);//设置搜索方法
	ne.setRadiusSearch (Radius);// 设置搜索半径
	ne.compute (*cloud_with_normals);//计算并输出法向量
	std::cerr << ">> Done: " << tt.toc () << " ms\n";

	// Set up a Conditional Euclidean Clustering class
	std::cerr << "Segmenting to clusters...\n", tt.tic ();
	pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);//创建条件聚类分割对象，并进行初始化。
	cec.setInputCloud (cloud_with_normals);//设置输入点集
	//用于选择不同条件函数
	switch(Method)
	{
	case 1:
		cec.setConditionFunction (&enforceIntensitySimilarity);
		break;
	case 2:
		cec.setConditionFunction (&enforceCurvatureOrIntensitySimilarity);
		break;
	case 3:
		cec.setConditionFunction (&customRegionGrowing);
		break;
	default:
		cec.setConditionFunction (&customRegionGrowing);
		break;
	}

	cec.setClusterTolerance (500.0);//设置聚类参考点的搜索距离
	cec.setMinClusterSize (cloud_with_normals->points.size () / 1000);//设置过小聚类的标准
	cec.setMaxClusterSize (cloud_with_normals->points.size () / 5);//设置过大聚类的标准
	cec.segment (*clusters);//获取聚类的结果，分割结果保存在点云索引的向量中
	cec.getRemovedClusters (small_clusters, large_clusters);//获取无效尺寸的聚类
	std::cerr << ">> Done: " << tt.toc () << " ms\n";
	
	for (int i = 0; i < small_clusters->size (); ++i)
		for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
			cloud_out->points[(*small_clusters)[i].indices[j]].intensity = -2.0;

	for (int i = 0; i < large_clusters->size (); ++i)
		for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
			cloud_out->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
	
	for (int i = 0; i < clusters->size (); ++i)
	{
		int label = rand () % 8;
		for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
			cloud_out->points[(*clusters)[i].indices[j]].intensity = label;
	}
	// Save the output point cloud
	if(0)
	{//可视化部分包含有错误。待修改！
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("点云库PCL学习教程第二版-条件分割方法")); 
		//View-Port1 
		int v1(0); 
		MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); //设置视口1的几何参数
		MView->setBackgroundColor (1, 0.2, 1); //设置视口1的背景
		MView->addText ("Before segmentation", 10, 10, "Before segmentation", v1); //为视口1添加标签
		int v2(0); 
		MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
		MView->setBackgroundColor (0.5, 0.5,0.5, v2); 
		MView->addText ("After segmentation", 10, 10, "After segmentation", v2); 
		//pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZI>::Ptr color_handler(new pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZI>());
		/*pcl::PCLPointCloud2::Ptr cloud;
		ColorHandlerPtr color_handler;
		pcl::fromPCLPointCloud2 (*cloud, *cloud_out);
		Eigen::Vector4f origin=Eigen::Vector4f::Identity();
		Eigen::Quaternionf orientation=Eigen::Quaternionf::Identity();

		color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud,"intensity"));*/
		MView->addPointCloud<pcl::PointXYZI>(cloud_in,"input",v1);//设置视口1的输入点云
		//MView->addPointCloud(cloud,color_handler,origin, orientation,"output",v2);
		MView->addPointCloud<pcl::PointXYZI>(cloud_out,"output",v2);
		MView->spin();
	}
	else
	{
		std::cerr << "Saving...\n", tt.tic ();
		pcl::io::savePCDFile ("output.pcd", *cloud_out);
		std::cerr << ">> Done: " << tt.toc () << " ms\n";
	}


	return (0);
}