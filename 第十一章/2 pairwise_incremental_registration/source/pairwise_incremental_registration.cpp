/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//这是一个辅助教程，因此我们可以负担全局变量
	//创建可视化工具
	pcl::visualization::PCLVisualizer *p;
	//定义左右视点
	int vp_1, vp_2;
//处理点云的方便的结构定义
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;
  PCD() : cloud (new PointCloud) {};
};
struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
//以< x, y, z, curvature >形式定义一个新的点
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    //定义尺寸值
    nr_dimensions_ = 4;
  }
  //覆盖copyToFloatArray方法来定义我们的特征矢量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
////////////////////////////////////////////////////////////////////////////////
/** 在可视化窗口的第一视点显示源点云和目标点云
*
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");
  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}
////////////////////////////////////////////////////////////////////////////////
/**在可视化窗口的第二视点显示源点云和目标点云
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");
  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");
  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
  p->spinOnce();
}
////////////////////////////////////////////////////////////////////////////////
/**加载一组我们想要匹配在一起的PCD文件
  * 参数argc是参数的数量 (pass from main ())
  *参数 argv 实际的命令行参数 (pass from main ())
  *参数models点云数据集的合成矢量
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  //假定第一个参数是实际测试模型
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // 至少需要5个字符长（因为.plot就有 5个字符）
    if (fname.size () <= extension.size ())
      continue;
    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    //检查参数是一个pcd文件
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      //加载点云并保存在总体的模型列表中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //从点云中移除NAN点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
      models.push_back (m);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
/**匹配一对点云数据集并且返还结果
  *参数 cloud_src 是源点云
  *参数 cloud_src 是目标点云
  *参数output输出的配准结果的源点云
  *参数final_transform是在来源和目标之间的转换
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  //为了一致性和高速的下采样
  //注意：为了大数据集需要允许这项
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);
    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }
  //计算曲面法线和曲率
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);
  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
  //
  //举例说明我们自定义点的表示（以上定义）
  MyPointRepresentation point_representation;
  //调整'curvature'尺寸权重以便使它和x, y, z平衡
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
  //
  // 配准
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  //将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
  //注意：根据你的数据集大小来调整
  reg.setMaxCorrespondenceDistance (0.1);  
  //设置点表示
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);
  //
  //在一个循环中运行相同的最优化并且使结果可视化
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);
    //为了可视化的目的保存点云
    points_with_normals_src = reg_result;
    //估计
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);
		//在每一个迭代之间累积转换
    Ti = reg.getFinalTransformation () * Ti;
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
      prev = reg.getLastIncrementalTransformation ();
    //可视化当前状态
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
	//
  // 得到目标点云到源点云的变换
  targetToSource = Ti.inverse();
  //
  //把目标点云转换回源框架
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  p->removePointCloud ("source");
  p->removePointCloud ("target");
  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();
  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  //添加源点云到转换目标
  *output += *cloud_src;
    final_transform = targetToSource;
 }
/* ---[ */
int main (int argc, char** argv)
{
  // 加载数据
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);
  //检查用户输入
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    PCL_INFO ("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd", argv[0]);
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());
    //创建一个PCL可视化对象
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;
    target = data[i].cloud;
    //添加可视化数据
    showCloudsLeft(source, target);
    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);
    //把当前的两两配对转换到全局变换
    pcl::transformPointCloud (*temp, *result, GlobalTransform);
    //update the global transform更新全局变换
    GlobalTransform = pairTransform * GlobalTransform;
		//保存配准对，转换到第一个点云框架中
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);
  }
}
/* ]--- */
