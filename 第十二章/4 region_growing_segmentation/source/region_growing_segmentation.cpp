#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <windows.h>
#include <stdio.h>
#include <psapi.h>
void PrintMemoryInfo( )
{
    HANDLE hProcess;
    PROCESS_MEMORY_COUNTERS pmc;

	hProcess=GetCurrentProcess();
    printf( "\nProcess ID: %u\n", hProcess );

    // Print information about the memory usage of the process.
	//输出进程使用的内存信息
   
    if (NULL == hProcess)
        return;

    if ( GetProcessMemoryInfo( hProcess, &pmc, sizeof(pmc)) )
    {
        printf( "\tPageFaultCount: 0x%08X\n", pmc.PageFaultCount );
        printf( "\tPeakWorkingSetSize: 0x%08X\n", 
                  pmc.PeakWorkingSetSize );
        printf( "\tWorkingSetSize: 0x%08X\n", pmc.WorkingSetSize );
        printf( "\tQuotaPeakPagedPoolUsage: 0x%08X\n", 
                  pmc.QuotaPeakPagedPoolUsage );
        printf( "\tQuotaPagedPoolUsage: 0x%08X\n", 
                  pmc.QuotaPagedPoolUsage );
        printf( "\tQuotaPeakNonPagedPoolUsage: 0x%08X\n", 
                  pmc.QuotaPeakNonPagedPoolUsage );
        printf( "\tQuotaNonPagedPoolUsage: 0x%08X\n", 
                  pmc.QuotaNonPagedPoolUsage );
        printf( "\tPagefileUsage: 0x%08X\n", pmc.PagefileUsage ); 
        printf( "\tPeakPagefileUsage: 0x%08X\n", 
                  pmc.PeakPagefileUsage );
    }

    CloseHandle( hProcess );
}

using namespace pcl::console;
int
main (int argc, char** argv)
{

	if(argc<2)
	{
		std::cout<<".exe xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05"<<endl;

		return 0;
	}//如果输入参数小于1个，输出提示
	time_t start,end,diff[5],option;
	start = time(0); 
	int KN_normal=50; //设置默认输入参数
	bool Bool_Cuting=false;//设置默认输入参数
	float far_cuting=10,near_cuting=0,SmoothnessThreshold=30.0,CurvatureThreshold=0.05;//设置默认输入参数
	parse_argument (argc, argv, "-kn", KN_normal);
	parse_argument (argc, argv, "-bc", Bool_Cuting);
	parse_argument (argc, argv, "-fc", far_cuting);
	parse_argument (argc, argv, "-nc", near_cuting);
	parse_argument (argc, argv, "-st", SmoothnessThreshold);
	parse_argument (argc, argv, "-ct", CurvatureThreshold);//设置输入参数方式
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}// 加载输入点云数据
	end = time(0); 
	diff[0] = difftime (end, start); 
	PCL_INFO ("\Loading pcd file takes(seconds): %d\n", diff[0]); 
	//Noraml estimation step(1 parameter)
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);//创建一个指向kd树搜索对象的共享指针
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//创建法线估计对象
	normal_estimator.setSearchMethod (tree);//设置搜索方法
	normal_estimator.setInputCloud (cloud);//设置法线估计对象输入点集
	normal_estimator.setKSearch (KN_normal);// 设置用于法向量估计的k近邻数目
	normal_estimator.compute (*normals);//计算并输出法向量
	end = time(0); 
	diff[1] = difftime (end, start)-diff[0]; 
	PCL_INFO ("\Estimating normal takes(seconds): %d\n", diff[1]); 
	//optional step: cutting the part are far from the orignal point in Z direction.2 parameters
	pcl::IndicesPtr indices (new std::vector <int>);//创建一组索引
	if(Bool_Cuting)//判断是否需要直通滤波
	{

		pcl::PassThrough<pcl::PointXYZ> pass;//设置直通滤波器对象
		pass.setInputCloud (cloud);//设置输入点云
		pass.setFilterFieldName ("z");//设置指定过滤的维度
		pass.setFilterLimits (near_cuting, far_cuting);//设置指定纬度过滤的范围
		pass.filter (*indices);//执行滤波，保存滤波结果在上述索引中
	}


	// 区域生长算法的5个参数
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;//创建区域生长分割对象
	reg.setMinClusterSize (50);//设置一个聚类需要的最小点数
	reg.setMaxClusterSize (1000000);//设置一个聚类需要的最大点数
	reg.setSearchMethod (tree);//设置搜索方法
	reg.setNumberOfNeighbours (30);//设置搜索的临近点数目
	reg.setInputCloud (cloud);//设置输入点云
	if(Bool_Cuting)reg.setIndices (indices);//通过输入参数设置，确定是否输入点云索引
	reg.setInputNormals (normals);//设置输入点云的法向量
	reg.setSmoothnessThreshold (SmoothnessThreshold / 180.0 * M_PI);//设置平滑阈值
	reg.setCurvatureThreshold (CurvatureThreshold);//设置曲率阈值

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);//获取聚类的结果，分割结果保存在点云索引的向量中。
	end = time(0); 
	diff[2] = difftime (end, start)-diff[0]-diff[1]; 
	PCL_INFO ("\Region growing takes(seconds): %d\n", diff[2]); 

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;//输出聚类的数量
	std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;//输出第一个聚类的数量
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	/* int counter = 0;
	while (counter < clusters[0].indices.size ())
	{
	std::cout << clusters[0].indices[counter] << ", ";
	counter++;
	if (counter % 10 == 0)
	std::cout << std::endl;
	}
	std::cout << std::endl;
	*/
	PrintMemoryInfo();
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("点云库PCL学习教程第二版-区域增长分割方法");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}//进行可视化

	return (0);
}