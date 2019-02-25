#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
int
main (int argc, char** argv)
{// 将一个适当类型的输入文件加载到对象PointCloud中
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // 加载bun0.pcd文件，加载的文件在 PCL的测试数据中是存在的 
  pcl::io::loadPCDFile ("table_scene_lms400_downsampled.pcd", *cloud);
  // 创建一个KD树
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
  pcl::PointCloud<pcl::PointNormal> mls_points;
  // 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  //设置参数
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  // 曲面重建
  mls.process (mls_points);
  // 保存结果
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
}	
