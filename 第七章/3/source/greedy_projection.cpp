#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/surface/impl/organized_fast_mesh.hpp> 
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
using namespace pcl::console;
int main (int argc, char** argv) {


	// Generate the data
	if (argc<2)
	{

		print_error ("Syntax is: %s input.pcd -w 640 -h 480 -cx 320 -cy 240 -fx 525 -fy 525 -type 0 -size 2\n", argv[0]);
		print_info ("  where options are:\n");
		print_info ("                     -w X = width of detph iamge ");

		return -1;
	}
	std::string filename = argv[1];

	int width=640,height=480,size=2,type=0;
	float fx=525,fy=525,cx=320,cy=240;

	parse_argument (argc, argv, "-w", width);
	parse_argument (argc, argv, "-h", height);
	parse_argument (argc, argv, "-cx", cx);
	parse_argument (argc, argv, "-cy", cy);
	parse_argument (argc, argv, "-fx", fx);
	parse_argument (argc, argv, "-fy", fy);
	parse_argument (argc, argv, "-type", type);
	parse_argument (argc, argv, "-size", size);
	//convert unorignized point cloud to orginized point cloud begin
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (filename, *cloud);
	print_info ("Read pcd file successfully\n");
	Eigen::Affine3f sensorPose;
	sensorPose.setIdentity(); 
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;

	pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
	rangeImage->createFromPointCloudWithFixedSize(*cloud,width,height,cx,cy,fx,fy,sensorPose,coordinate_frame);
	std::cout << rangeImage << "\n";
	//convert unorignized point cloud to orginized point cloud end


	//viusalization of range image
	pcl::visualization::RangeImageVisualizer range_image_widget ("点云库PCL从入门到精通");
	range_image_widget.showRangeImage (*rangeImage);
	range_image_widget.setWindowTitle("点云库PCL从入门到精通");
	//triangulation based on range image
	pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
	pcl::search::KdTree<pcl::PointWithRange>::Ptr tree (new pcl::search::KdTree<pcl::PointWithRange>);
	tree->setInputCloud(rangeImage);
	pcl::PolygonMesh triangles;
	tri->setTrianglePixelSize(size);
	tri->setInputCloud(rangeImage);
	tri->setSearchMethod(tree);
	tri->setTriangulationType((pcl::OrganizedFastMesh<pcl::PointWithRange>::TriangulationType)type);
	tri->reconstruct(triangles);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("点云库PCL从入门到精通"));
	viewer->setBackgroundColor(0.5,0.5,0.5);
	viewer->addPolygonMesh(triangles,"tin");
	viewer->addCoordinateSystem();
	while (!range_image_widget.wasStopped ()&&!viewer->wasStopped())
	{
		range_image_widget.spinOnce ();

		pcl_sleep (0.01);
		viewer->spinOnce ();

	}
}