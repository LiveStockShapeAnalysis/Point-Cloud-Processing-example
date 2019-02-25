#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
	PointCloudT &adjacent_supervoxel_centers,
	std::string supervoxel_name,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


int
	main (int argc, char ** argv)
{
	if (argc < 2)
	{
		pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
			"--NT Dsables the single cloud transform \n"
			"-v <voxel resolution>\n-s <seed resolution>\n"
			"-c <color weight> \n-z <spatial weight> \n"
			"-n <normal_weight>\n", argv[0]);
		return (1);
	}


	PointCloudT::Ptr cloud = boost::make_shared <PointCloudT> ();
	pcl::console::print_highlight ("Loading point cloud...\n");
	if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
	{
		pcl::console::print_error ("Error loading cloud file!\n");
		return (1);
	}
	cout<<"point size of input: "<<cloud->size()<<endl;

	bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");

	float voxel_resolution = 0.008f;
	bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
	if (voxel_res_specified)
		pcl::console::parse (argc, argv, "-v", voxel_resolution);

	float seed_resolution = 0.1f;
	bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
	if (seed_res_specified)
		pcl::console::parse (argc, argv, "-s", seed_resolution);

	float color_importance = 0.2f;
	if (pcl::console::find_switch (argc, argv, "-c"))
		pcl::console::parse (argc, argv, "-c", color_importance);

	float spatial_importance = 0.4f;
	if (pcl::console::find_switch (argc, argv, "-z"))
		pcl::console::parse (argc, argv, "-z", spatial_importance);

	float normal_importance = 1.0f;
	if (pcl::console::find_switch (argc, argv, "-n"))
		pcl::console::parse (argc, argv, "-n", normal_importance);

	//////////////////////////////  //////////////////////////////
	////// This is how to use supervoxels
	//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
	if (disable_transform)
		super.setUseSingleCameraTransform (false);
	super.setInputCloud (cloud);
	super.setColorImportance (color_importance);
	super.setSpatialImportance (spatial_importance);
	super.setNormalImportance (normal_importance);

	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
	//该单映射容器以标签为键值存储所有超体素
	pcl::console::print_highlight ("Extracting supervoxels!\n");
	super.extract (supervoxel_clusters);
	pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("点云库PCL学习教程第二版-超体素分割"));
	viewer->setBackgroundColor (1,1,1);

	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
	cout<<"voxel centroids: "<<voxel_centroid_cloud->size()<<endl;
	if(0)
	{//对于体素中心的可视化和保存，基本就是对原始数据的空间均匀下采样
		viewer->addPointCloud<PointT>(voxel_centroid_cloud,"voxel centroids");
		pcl::io::savePCDFile("voxel_centroids.pcd",*voxel_centroid_cloud);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4, "voxel centroids");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.5, "voxel centroids");
	}


	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
	if(1)
	{//超体素分割结果显示与保存
		pcl::io::savePCDFile("labeled_voxels.pcd",*labeled_voxel_cloud);
		viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
		cout<<"labeled voxels: "<<labeled_voxel_cloud->size()<<endl;
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3, "labeled voxels");
		// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
	}

	PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
	//
	if(0)//超体素对应的法线特征可视化
		viewer->addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

	pcl::console::print_highlight ("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency (supervoxel_adjacency);
	cout<<"size of supervoxel_adjacency: "<<supervoxel_adjacency.size()<<endl;

	//遍历多重映射容器构造邻接图
	std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
	for ( ; label_itr != supervoxel_adjacency.end (); )
	{
		//获取标签值
		uint32_t supervoxel_label = label_itr->first;
		//根据标签索引到该超体素
		pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

		//遍历该超体素相邻超体素并以其相邻超体素中心为点集构造点云，用于后续可视化，这里的相邻超体素在多重映射容器中具有相同的键值
		PointCloudT adjacent_supervoxel_centers;
		std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
		for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
		{
			pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
			adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
		}
		//
		std::stringstream ss;
		ss << "supervoxel_" << supervoxel_label;
		//cout<<ss.str()<<endl;
		//绘制该超体素与其相邻超体素子图
		addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
		//使迭代器指向下一个标签。
		label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
	}

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce();
	}
	return (0);
}

void
	addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
	PointCloudT &adjacent_supervoxel_centers,
	std::string supervoxel_name,
	boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{

	int i=0;
	//Iterate through all adjacent points, and add a center point to adjacent point pair
	PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
	for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
	{
		std::stringstream ss;
		ss<<supervoxel_name<<i;
		viewer->addLine(supervoxel_center,*adjacent_itr,ss.str());

		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,ss.str());
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,ss.str());
		ss<<supervoxel_name<<i;
		viewer->addSphere(supervoxel_center,0.008,0,0,255,ss.str());
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD,ss.str());
		//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.9,ss.str());
		i++;
	}

}


