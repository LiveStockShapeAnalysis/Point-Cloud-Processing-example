#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(argv[1],*cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, -0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.15, 0.1,0 ));

	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(boundingbox_ptr);
	hull.setDimension(2);
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surface_hull, polygons);

	pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter;
	bb_filter.setDim(2);
	bb_filter.setInputCloud(cloud);
	bb_filter.setHullIndices(polygons);
	bb_filter.setHullCloud(surface_hull);
	bb_filter.filter(*objects);
	std::cout << objects->size() << std::endl;

	//visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v (new pcl::visualization::PCLVisualizer ("crophull display"));
	for_visualizer_v->setBackgroundColor(255,255,255);

	int v1(0);
	for_visualizer_v->createViewPort (0.0, 0.0, 0.33, 1, v1);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v1);
	for_visualizer_v->addPointCloud (cloud,"cloud",v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline1",v1);

	int v2(0);
	for_visualizer_v->createViewPort (0.33, 0.0, 0.66, 1, v2);	
	for_visualizer_v->setBackgroundColor (255, 255, 255, v2);
	for_visualizer_v->addPointCloud (surface_hull,"surface_hull",v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"surface_hull");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"surface_hull");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline",v2);

	int v3(0);
	for_visualizer_v->createViewPort (0.66, 0.0, 1, 1, v3);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v3);
	for_visualizer_v->addPointCloud (objects,"objects",v3);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"objects");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"objects");

	while (!for_visualizer_v->wasStopped())
	{

		for_visualizer_v->spinOnce(1000);
	}
	system("pause");
}
