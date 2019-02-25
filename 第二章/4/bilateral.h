#ifndef PCL_FILTERS_BILATERAL_H_
 #define PCL_FILTERS_BILATERAL_H_
 #include <pcl/filters/filter.h>
 #include <pcl/kdtree/kdtree.h>

namespace pcl
 {
template<typename PointT>
classBilateralFilter:public Filter<PointT>
   {
using Filter<PointT>::input_;
typedeftypename Filter<PointT>::PointCloud PointCloud;
typedeftypename pcl::KdTree<PointT>::Ptr KdTreePtr;

public:
       BilateralFilter () : sigma_s_ (0),
                            sigma_r_ (std::numeric_limits<double>::max ())
       {
       }


void
applyFilter (PointCloud &output);

double
computePointWeight (constint pid, const std::vector<int>&indices, const std::vector<float>&distances);

void
setSigmaS (constdouble sigma_s)
       {
         sigma_s_ = sigma_s;
       }

double
getSigmaS ()
       {
return (sigma_s_);
       }

void
setSigmaR (constdouble sigma_r)
       {
         sigma_r_ = sigma_r;
       }

double
getSigmaR ()
       {
return (sigma_r_);
       }

void
setSearchMethod (const KdTreePtr &tree)
       {
         tree_ = tree;
       }


private:

inlinedouble
kernel (double x, double sigma)
       {
return (exp (- (x*x)/(2*sigma*sigma)));
       }

double sigma_s_;
double sigma_r_;
       KdTreePtr tree_;
   };
 }

 #endif // PCL_FILTERS_BILATERAL_H_
