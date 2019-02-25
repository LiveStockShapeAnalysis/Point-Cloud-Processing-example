#include <pcl/point_types.h>
 #include <pcl/impl/instantiate.hpp>
 #include <pcl/filters/bilateral.h>
 #include <pcl/filters/impl/bilateral.hpp>
PCL_INSTANTIATE(BilateralFilter,(pcl::PointXYZI)(pcl::PointXYZINormal));
