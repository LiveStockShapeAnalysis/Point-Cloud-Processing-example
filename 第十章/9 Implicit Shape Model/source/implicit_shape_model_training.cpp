#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>

int
main (int argc, char** argv)
{
  if (argc == 0)
  { std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << "class1.pcd class1_label(int) class2.pcd class2_label" << std::endl << std::endl;
   return (-1);
  }
  unsigned int number_of_training_clouds = (argc - 1) / 2;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> training_clouds;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> training_normals;
  std::vector<unsigned int> training_classes;

  for (unsigned int i_cloud = 0; i_cloud < number_of_training_clouds - 1; i_cloud++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[i_cloud * 2 + 1], *tr_cloud) == -1 )
      return (-1);

    pcl::PointCloud<pcl::Normal>::Ptr tr_normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
    normal_estimator.setInputCloud (tr_cloud);
    normal_estimator.compute (*tr_normals);

    unsigned int tr_class = static_cast<unsigned int> (strtol (argv[i_cloud * 2 + 2], 0, 10));

    training_clouds.push_back (tr_cloud);
    training_normals.push_back (tr_normals);
    training_classes.push_back (tr_class);
  }

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);
 
  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator);
  ism.setTrainingClouds (training_clouds);
  ism.setTrainingNormals (training_normals);
  ism.setTrainingClasses (training_classes);
  ism.setSamplingSize (2.0f);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model = boost::shared_ptr<pcl::features::ISMModel>
    (new pcl::features::ISMModel);
  ism.trainISM (model);

  std::string file ("trained_ism_model.txt");
  model->saveModelToFile (file);

  std::cout << "trained_ism_model.txt is the output of training stage. You can use the trained_ism_model.txt in the classification stage" << std::endl << std::endl;

  return (0);
}