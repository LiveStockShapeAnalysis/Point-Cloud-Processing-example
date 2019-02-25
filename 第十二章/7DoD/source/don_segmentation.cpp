/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 * @ÐÞ¸Ä 2015-6-16
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <pcl/features/don.h>
// for visualization
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr input_, std::vector <pcl::PointIndices> clusters_,float r,float g,float b)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = r;
      point.g = g;
      point.b = b;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

int
main (int argc, char *argv[])
{
	int VISUAL=1,SAVE=0;//0 indicate shows nothing, 1 indicate shows very step output 2 only shows the final results
  ///The smallest scale to use in the DoN filter.
  double scale1,mean_radius;

  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;

  if (argc < 6)
  {
    cerr << "usage: " << argv[0] << " inputfile smallscale(5) largescale(10) threshold(0.1) segradius(1.5) VISUAL(1) SAVE(0)" << endl;
	cerr << "usage: "<<"smallscale largescale  segradius :multiple with mean radius of point cloud "<< endl;
    exit (EXIT_FAILURE);
  }

  /// the file to read from.
  string infile = argv[1];
  /// small scale
  istringstream (argv[2]) >> scale1;
  /// large scale
  istringstream (argv[3]) >> scale2;
  istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  istringstream (argv[5]) >> segradius;   // threshold for radius segmentation
  istringstream (argv[6]) >> VISUAL;
   istringstream (argv[7]) >> SAVE;
  // Load cloud in blob format
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::io::loadPCDFile (infile.c_str (), *cloud);
   // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);
 //caculate the mean radius of cloud and mutilply with corresponding input
  {
	  int size_cloud=cloud->size();
	  int step=size_cloud/10;
	  double total_distance=0;
	  int i,j=1;
	  for(i=0;i<size_cloud;i+=step,j++)
	  {
		  std::vector<int> pointIdxNKNSearch(2);
		  std::vector<float> pointNKNSquaredDistance(2);
		  tree->nearestKSearch(cloud->points[i],2,pointIdxNKNSearch,pointNKNSquaredDistance);
		  total_distance+=pointNKNSquaredDistance[1]+pointNKNSquaredDistance[0];
	  }
	  mean_radius=sqrt((total_distance/j));
	  cout<<"mean radius of cloud is£º "<<mean_radius<<endl;
	  scale1*=mean_radius;
	  scale2*=mean_radius;
	  segradius*=mean_radius;
  }
 

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);
  //visualize the normals
  if(VISUAL=1)
  {
	  cout << "click q key to quit the visualizer and continue£¡£¡" << endl;
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Showing normals with different scale")); 
	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud, 0,255,0); 
	  int v1(0),v2(0); 
	  MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
	  MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
	  MView->setBackgroundColor (1,1,1); 
	  MView->addPointCloud (cloud, green, "small_scale", v1); 
	  MView->addPointCloud (cloud, green, "large_scale", v2);
	  MView->addPointCloudNormals<pcl::PointXYZRGB,pcl::PointNormal>(cloud,normals_small_scale,100,mean_radius*10,"small_scale_normal");
	  MView->addPointCloudNormals<pcl::PointXYZRGB,pcl::PointNormal>(cloud,normals_large_scale,100,mean_radius*10,"large_scale_normal");
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"small_scale",v1);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"small_scale",v1);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"large_scale",v1);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"large_scale",v1);
	  MView->spin();

  }
  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);
  

  //print some differencense of curvature
  {
	   cout << "You may have some sense about the input threshold£¨curvature£© next time for your data" << endl;
	  int size_cloud=doncloud->size();
	  int step=size_cloud/10;
	  for(int i=0;i<size_cloud;i+=step)cout << " "<<doncloud->points[i].curvature<<" "<< endl;

  }

   //show the differences of curvature with both large and small scale 
  if(VISUAL=1)
  {
	  cout << "click q key to quit the visualizer and continue£¡£¡" << endl;
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Showing the difference of curvature of two scale")); 
	  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(doncloud,"curvature"); 
	  MView->setBackgroundColor (1,1,1); 
	  MView->addPointCloud (doncloud, handler_k); 
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5);
	  MView->spin();
  }

  
  // Save DoN features
  pcl::PCDWriter writer;
   if(SAVE==1) writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 


  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  if(SAVE==1)writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 


   //show the results of keeping relative small curvature points 
  if(VISUAL==1)
  {
	  cout << "click q key to quit the visualizer and continue£¡£¡" << endl;
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Showing the results of keeping relative small curvature points")); 
	  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(doncloud,"curvature"); 
	  MView->setBackgroundColor (1,1,1); 
	  MView->addPointCloud (doncloud, handler_k); 
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5);
	  MView->spin();
  }

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);
  if(VISUAL==1)
  {//visualize the clustering results
	  pcl::PointCloud <pcl::PointXYZ>::Ptr tmp_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	  copyPointCloud<pcl::PointNormal,pcl::PointXYZ>(*doncloud,*tmp_xyz);
	  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = getColoredCloud (tmp_xyz,cluster_indices,0,255,0);

	  cout << "click q key to quit the visualizer and continue£¡£¡" << endl;
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("visualize the clustering results")); 
	 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbps(colored_cloud); 
	  MView->setBackgroundColor (1,1,1); 
	  MView->addPointCloud (colored_cloud, rgbps); 
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
	  MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5);
	  MView->spin();

  }
  if(SAVE==1)
  {
	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
	  {
		  pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
		  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		  {
			  cloud_cluster_don->points.push_back (doncloud->points[*pit]);
		  }

		  cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
		  cloud_cluster_don->height = 1;
		  cloud_cluster_don->is_dense = true;

		  //Save cluster
		  cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
		  stringstream ss;
		  ss << "don_cluster_" << j << ".pcd";
		  writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
	  }

  }
  
}