#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/histogram_visualizer.h>
#include<pcl/visualization/pcl_plotter.h>
int main (int argc, char** argv)
{
	if (argc != 4)
		return (-1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
		return (-1);

	pcl::PointIndicesPtr indices = boost::shared_ptr <pcl::PointIndices> (new pcl::PointIndices ());
	std::ifstream indices_file;
	indices_file.open (argv[2], std::ifstream::in);
	for (std::string line; std::getline (indices_file, line);)
	{
		std::istringstream in (line);
		unsigned int index = 0;
		in >> index;
		indices->indices.push_back (index - 1);
	}
	indices_file.close ();

	std::vector <pcl::Vertices> triangles;
	std::ifstream triangles_file;
	triangles_file.open (argv[3], std::ifstream::in);
	for (std::string line; std::getline (triangles_file, line);)
	{
		pcl::Vertices triangle;
		std::istringstream in (line);
		unsigned int vertex = 0;
		in >> vertex;
		triangle.vertices.push_back (vertex - 1);
		in >> vertex;
		triangle.vertices.push_back (vertex - 1);
		in >> vertex;
		triangle.vertices.push_back (vertex - 1);
		triangles.push_back (triangle);
	}

	float support_radius = 0.0285f;
	unsigned int number_of_partition_bins = 5;
	unsigned int number_of_rotations = 3;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
	search_method->setInputCloud (cloud);

	pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
	feature_estimator.setSearchMethod (search_method);
	feature_estimator.setSearchSurface (cloud);
	feature_estimator.setInputCloud (cloud);
	feature_estimator.setIndices (indices);
	feature_estimator.setTriangles (triangles);
	feature_estimator.setRadiusSearch (support_radius);
	feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
	feature_estimator.setNumberOfRotations (number_of_rotations);
	feature_estimator.setSupportRadius (support_radius);

	pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
	feature_estimator.compute (*histograms);

	std::cout<<histograms->header<<endl;
	std::string title="点云库PCL学习教程第二版-ROPS特征描述子";
	pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter (title.c_str());//此处应该有个bug，通过构建函数传递的窗口名不起作用。
	plotter->setWindowName(title);//所以用该函数设置窗口名。
	plotter->setShowLegend (true);
	plotter->addFeatureHistogram<pcl::Histogram <135>>(*histograms,135,"rops");//
	//显示第0个索引对应点的特征直方图,如果要显示其他索引，本来对于PCL中用POINT_CLOUD_REGISTER_POINT_STRUCT注册的结构体，例如fpfh等特征，就可以利用函数
    /*pcl::visualization::PCLPlotter::addFeatureHistogram (
    const pcl::PointCloud<PointT> &cloud, 
    const std::string &field_name,
    const int index, 
    const std::string &id, int win_width, int win_height),但本例中的pcl::Histogram <135>是没用POINT_CLOUD_REGISTER_POINT_STRUCT注册过，即没有field_name，简单的显示方式就是把想显示
	的点对应的特征向量，作为单独一个新的点云来对待，就可以显示*/
	plotter->spin();
	return (0);
}