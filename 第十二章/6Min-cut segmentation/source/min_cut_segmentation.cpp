#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
using namespace pcl::console;
int main (int argc, char** argv)
{
	if(argc<2)
	{
		std::cout<<".exe xx.pcd -bc 1 -fc 1.0 -nc 0 -cx 68.97 -cy -18.55 -cz 0.57 -s 0.25 -r 3.0433856 -non 14 -sw 0.5"<<endl;

		return 0;
	}//如果输入参数小于1个，输出提示信息
	time_t start,end,diff[5],option;
	start = time(0); 
	int NumberOfNeighbours=14;//设置默认参数
	bool Bool_Cuting=false;//设置默认参数
	float far_cuting=1,near_cuting=0,C_x=0.071753,C_y= -0.309913,C_z=1.603000,Sigma=0.25,Radius=0.8,SourceWeight=0.5;//设置默认输入参数
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);// 创建一个PointCloud <pcl::PointXYZRGB>共享指针并进行实例化
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1 )// 加载点云数据
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	parse_argument (argc, argv, "-bc", Bool_Cuting);
	parse_argument (argc, argv, "-fc", far_cuting);
	parse_argument (argc, argv, "-nc", near_cuting);

	parse_argument (argc, argv, "-cx", C_x);
	parse_argument (argc, argv, "-cy", C_y);
	parse_argument (argc, argv, "-cz", C_z);

	parse_argument (argc, argv, "-s", Sigma);
	parse_argument (argc, argv, "-r", Radius);
	parse_argument (argc, argv, "-non", NumberOfNeighbours);
	parse_argument (argc, argv, "-sw", SourceWeight);//设置输入参数方式

	pcl::IndicesPtr indices (new std::vector <int>);//创建一组索引
	if(Bool_Cuting)//判断是否需要直通滤波
	{
		pcl::PassThrough<pcl::PointXYZ> pass;//设置直通滤波器对象
		pass.setInputCloud (cloud);//设置输入点云
		pass.setFilterFieldName ("z");//设置指定过滤的维度
		pass.setFilterLimits (near_cuting, far_cuting);//设置指定纬度过滤的范围
		pass.filter (*indices);//执行滤波，保存滤波结果在上述索引中

	}
	pcl::MinCutSegmentation<pcl::PointXYZ> seg;//创建一个PointXYZRGB类型的区域生长分割对象
	seg.setInputCloud (cloud);//设置输入点云
	if(Bool_Cuting)seg.setIndices (indices);//设置输入点云的索引

	pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());//创建一个PointCloud <pcl::PointXYZRGB>共享指针并进行实例化
	pcl::PointXYZ point;//定义中心点并赋值

	point.x =C_x;
	point.y = C_y;
	point.z = C_z;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints (foreground_points);//输入前景点云（目标物体）的中心点

	seg.setSigma (Sigma);//设置平滑成本的Sigma值
	seg.setRadius (Radius);//设置背景惩罚权重的半径
	seg.setNumberOfNeighbours (NumberOfNeighbours);//设置临近点数目
	seg.setSourceWeight (SourceWeight);//设置前景惩罚权重

	std::vector <pcl::PointIndices> clusters;
	seg.extract (clusters);//获取分割的结果，分割结果保存在点云索引的向量中。

	std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;//计算并输出分割期间所计算出的流值

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();//对前景点赋予红色，对背景点赋予白色。
	pcl::visualization::PCLVisualizer viewer ("点云库PCL学习教程第二版-最小割分割方法");
	viewer.addPointCloud(colored_cloud);
	viewer.addSphere(point,Radius,122,122,0,"sphere");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.2,"sphere");
	
	while (!viewer.wasStopped ())
	{
		viewer.spin();
	}//进行可视化

	return (0);
}