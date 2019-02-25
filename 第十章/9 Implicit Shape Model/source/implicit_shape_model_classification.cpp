#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

int
main (int argc, char** argv)
{
   if (argc == 0)
  { std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " test_scene.pcd class1_label(int)" << std::endl << std::endl;
  std::cout << "Where the parameter class1_label is the object you want to be segmented and recognized" << std::endl << std::endl;
   return (-1);
  }


  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator);
  ism.setSamplingSize (2.0f);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model = boost::shared_ptr<pcl::features::ISMModel>
    (new pcl::features::ISMModel);
  std::string file ("trained_ism_model.txt");
  model->loadModelFromfile (file);
  unsigned int testing_class = static_cast<unsigned int> (strtol (argv[2], 0, 10));
  pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *testing_cloud) == -1 )
    return (-1);

  pcl::PointCloud<pcl::Normal>::Ptr testing_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
  normal_estimator.setInputCloud (testing_cloud);
  normal_estimator.compute (*testing_normals);

  boost::shared_ptr<pcl::features::ISMVoteList<pcl::PointXYZ> > vote_list = ism.findObjects (
    model,
    testing_cloud,
    testing_normals,
    testing_class);

  double radius = model->sigmas_[testing_class] * 10.0;
  double sigma = model->sigmas_[testing_class];
  std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
  vote_list->findStrongestPeaks (strongest_peaks, testing_class, radius, sigma);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  pcl::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;

  /*for (size_t i_point = 0; i_point < testing_cloud->points.size (); i_point++)
  {
    point.x = testing_cloud->points[i_point].x;
    point.y = testing_cloud->points[i_point].y;
    point.z = testing_cloud->points[i_point].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += testing_cloud->points.size ();
*/
  point.r = 255;
  point.g = 0;
  point.b = 0;
  for (size_t i_vote = 0; i_vote < strongest_peaks.size (); i_vote++)
  {
    point.x = strongest_peaks[i_vote].x;
    point.y = strongest_peaks[i_vote].y;
    point.z = strongest_peaks[i_vote].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += strongest_peaks.size ();

  pcl::visualization::PCLVisualizer viewer ("点云库PCL学习教程第二版-隐式形状模型");
  viewer.setBackgroundColor(1,1,1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorh(testing_cloud,30,200,30);
  viewer.addPointCloud(testing_cloud,colorh,"test_data");
  viewer.addPointCloud (colored_cloud,"centors");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"centors");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"test_data");
  while (!viewer.wasStopped ())
  {
	  viewer.spin();
  }

  return (0);
}