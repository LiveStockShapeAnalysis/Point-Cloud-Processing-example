/* \author Kripasindhu Sarkar */

#include<pcl/visualization/pcl_plotter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/normal_space.h>
#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/histogram_visualizer.h>


#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  //for abs()

using namespace std;
using namespace pcl::visualization;
using namespace pcl::console;
void
generateData (double *ax, double *acos, double *asin, int numPoints)
{
  double inc = 7.5 / (numPoints - 1);
  for (int i = 0; i < numPoints; ++i)
  {
    ax[i] = i*inc;
    acos[i] = cos (i * inc);
    asin[i] = sin (i * inc);
  }
}

//.....................回调函数....................
double
step (double val)
{
  if (val > 0)
    return (double) (int) val;
  else
    return (double) ((int) val - 1);
}

double
identity_i (double val)
{
  return val;
}


int
main (int argc, char * argv [])
{
		if(argc<2)
	{
		std::cout<<".exe source.pcd -r 0.005 -ds 5"<<endl;
		return 0;
	}
	float voxel_re=0.005,ds_N=5;
	parse_argument (argc, argv, "-r", voxel_re);// 设置点云分辨率
	parse_argument (argc, argv, "-ds", ds_N);// 设置半径
  

	//调节下采样的分辨率以保持数据处理的速度。
	// 下采样 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::io::loadPCDFile (argv[1], *cloud_src);	
	std::vector<int> indices1; 
	pcl::removeNaNFromPointCloud (*cloud_src, *cloud_src, indices1); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr ds_src (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::VoxelGrid<pcl::PointXYZ> grid; 
	grid.setLeafSize (voxel_re, voxel_re, voxel_re); 
	grid.setInputCloud (cloud_src); 
	grid.filter (*ds_src); 


	//计算法向量
	pcl::PointCloud<pcl::Normal>::Ptr norm_src (new pcl::PointCloud<pcl::Normal>); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>()); 
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; 
	PCL_INFO ("	Normal Estimation - Source \n");	
	ne.setInputCloud (ds_src); 
	ne.setSearchSurface (cloud_src); 
	ne.setSearchMethod (tree_src); 
	ne.setRadiusSearch (ds_N*2*voxel_re); 
	ne.compute (*norm_src); 

	// 提取关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt (new pcl::PointCloud<pcl::PointXYZ>); 
	grid.setLeafSize (ds_N*voxel_re,ds_N*voxel_re,ds_N*voxel_re); 
	grid.setInputCloud (ds_src); 
	grid.filter (*keypoints_src); 
       

	//Feature-Descriptor 
	PCL_INFO ("FPFH - Feature Descriptor\n"); 
	//FPFH	
	//FPFH Source 
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est_src; 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_src (new pcl::search::KdTree<pcl::PointXYZ>); 

	fpfh_est_src.setSearchSurface (ds_src);//输入完整点云数据
	fpfh_est_src.setInputCloud (keypoints_src); // 输入关键点
	fpfh_est_src.setInputNormals (norm_src); 
	fpfh_est_src.setRadiusSearch (2*ds_N*voxel_re);
	fpfh_est_src.setSearchMethod(tree_fpfh_src);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src (new pcl::PointCloud<pcl::FPFHSignature33>); 
	fpfh_est_src.compute (*fpfh_src); 



  //定义绘图器
  PCLPlotter *plotter = new PCLPlotter ("My Plotter"); 
  //设置特性
  plotter->setShowLegend (true);
  std::cout<<pcl::getFieldsList<pcl::FPFHSignature33>(*fpfh_src);
  plotter->addFeatureHistogram<pcl::FPFHSignature33>(*fpfh_src,"fpfh",5,"one_fpfh");
  //显示两秒
  plotter->setWindowSize (800, 600);
  plotter->spinOnce (30000000);
  plotter->clearPlots ();

  // 产生对应点对
  int numPoints = 69;
  double ax[100], acos[100], asin[100];
  generateData (ax, acos, asin, numPoints);

  //添加绘图数据
  plotter->addPlotData (ax, acos, numPoints, "cos");
  plotter->addPlotData (ax, asin, numPoints, "sin");

  //显示2s
  plotter->spinOnce (30000);
  plotter->clearPlots ();
  
  
  //...................绘制隐式函数....................

  //设置y轴范围
  plotter->setYRange (-10, 10);

  //定义多项式
  vector<double> func1 (1, 0);
  func1[0] = 1; //y = 1
  vector<double> func2 (3, 0);
  func2[2] = 1; //y = x^2

  plotter->addPlotData (std::make_pair (func1, func2), -10, 10, "y = 1/x^2", 100, vtkChart::POINTS);
  plotter->spinOnce (2000);

  plotter->addPlotData (func2, -10, 10, "y = x^2");
  plotter->spinOnce (2000);

  //回调函数
  plotter->addPlotData ( identity_i, -10, 10, "identity");
  plotter->spinOnce (2000);

  plotter->addPlotData (abs, -10, 10, "abs");
  plotter->spinOnce (2000);

  plotter->addPlotData (step, -10, 10, "step", 100, vtkChart::POINTS);
  plotter->spinOnce (2000);

  plotter->clearPlots ();

  //........................一个简单动画..............................

 
  vector<double> fsq (3, 0);
  fsq[2] = -100; //y = x^2
  while (plotter->wasStopped ())
  {
    if (fsq[2] == 100) fsq[2] = -100;
    fsq[2]++;
    char str[50];
    sprintf (str, "y = %dx^2", (int) fsq[2]);
    plotter->addPlotData (fsq, -10, 10, str);
	 plotter->setYRange (-1, 1);
    plotter->spinOnce (100);
    plotter->clearPlots ();
  }
  
  return 1;
}