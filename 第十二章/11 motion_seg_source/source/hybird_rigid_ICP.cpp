
	#include <boost/thread/thread.hpp>
	#include <boost/filesystem.hpp>
	#include <pcl/io/io.h>
	#include <pcl/io/pcd_io.h>
	#include <pcl/features/integral_image_normal.h>
	#include <pcl/visualization/cloud_viewer.h>
	#include <pcl/point_types.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/console/print.h>
	#include <pcl/console/parse.h>
	#include <pcl/console/time.h>
	#include <Eigen/StdVector>
	#include <Eigen/Geometry>
	#include <pcl/filters/extract_indices.h>
	#include <pcl/sample_consensus/ransac.h>
	#include <pcl/sample_consensus/sac_model_sphere.h>
	#include <boost/thread/thread.hpp>
	#include <pcl/ModelCoefficients.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/sample_consensus/method_types.h>
	#include <pcl/sample_consensus/model_types.h>
	#include <pcl/segmentation/sac_segmentation.h>
	#include <pcl/segmentation/extract_clusters.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/filter.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
	#include <pcl/registration/icp.h>
	#include <pcl/registration/icp_nl.h>
	#include <pcl/registration/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/common/angles.h>
	using namespace pcl::console;
	using pcl::visualization::PointCloudColorHandlerGenericField;
	using pcl::visualization::PointCloudColorHandlerCustom;
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
	double tstart, tstop, ttime;
	std::vector<std::string> pcd_files_;
	std::vector<boost::filesystem::path> pcd_paths_;
	std::string dir_;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
	int vp_1, vp_2,vp_3;
	int cidx=-100;
	void
	pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
	{
	static int k;
	std::string str;
	cidx = event.getPointIndex ();
	if (cidx == -1)
	return;
	pcl::PointXYZ picked_pt;
	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
	PCL_INFO ("Point index picked: %d  - [%f, %f, %f]\n", cidx, picked_pt.x, picked_pt.y, picked_pt.z);
	str=k;
	p->addSphere(picked_pt,0.03,1,0,0,str);
	k++;
  
	}

	void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
	{
	p->removePointCloud ("vp1_source");
	p->setBackgroundColor(255,255,255);
	PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud_source, 255, 0, 0);
	p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
	print_info ("visualization of source in first viewport,click q to continue\n");
	p-> spin();
	}
	void showCloudstmp(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
	{
    p->setBackgroundColor(255,255,255);
	PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud_source, 0, 0, 255);
	p->addPointCloud (cloud_source, src_h, "tmp", vp_3);
	print_info ("visualization of tempresult in viewport 3,click q to continue\n");
	p-> spin();
	p->removePointCloud ("tmp");
	}
	void showCloudstmp(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,Eigen::Matrix<float, 4, 1> &centroid_back,Eigen::Matrix<float, 4, 1> &centroid_forth)
	{
	p->setBackgroundColor(255,255,255);
	PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud_source, 0, 0, 255);
	p->addPointCloud (cloud_source, src_h, "tmpforthROI", vp_3);
	print_info ("visualization of tempresult in viewport 3,click q to continue\n");
	pcl::PointXYZ picked_ptf,picked_ptb;
	picked_ptf.x=centroid_forth[0];
	picked_ptf.y=centroid_forth[1];
	picked_ptf.z=centroid_forth[2];
	picked_ptb.x=centroid_back[0];
	picked_ptb.y=centroid_back[1];
	picked_ptb.z=centroid_back[2];
	p->addSphere(picked_ptb,0.03,0,1,0,"tmpforthsphere");
	p->addSphere(picked_ptf,0.03,0,1,0,"tmpbacksphere");
	p->spin();
	p->removePointCloud ("tmpforthROI");
	p->removeShape("tmpforthsphere");
	p->removeShape("tmpbacksphere");
	}

	void showCloudsRight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target)
	{
	p->setBackgroundColor(255,255,255);
	p->removePointCloud ("target");

	PointCloudColorHandlerCustom<PointT> src_h (cloud_target, 0, 255, 0);
	p->addPointCloud (cloud_target, src_h, "target", vp_2);
	print_info ("visualization of tempresult in viewport 2,click q to continue\n");
	p->spin();
	}
	int
	main (int argc, char** argv)
	{
		bool voxel_filter=true,del_plane=true,save_data=false;
		Eigen::Matrix4f toFirstT=Eigen::Matrix4f::Identity ();
		tstart = (double)clock()/CLOCKS_PER_SEC;
		print_info ("begin to have pcd file list\n");

		if (argc<2)
			{
				print_error ("Syntax is: %s input.pcd -dir E:\cow _paper_patents\mono kinect cover part\live Pig\continue\one \n", argv[0]);
				print_info ("  where options are:\n");
				print_info ("                     -dir X =directory of pcd sequences");
				return -1;
			}
			parse_argument (argc, argv, "-dir", dir_);
			pcd_files_.clear ();     
			pcd_paths_.clear ();    
	 
			//点云序列读取模块
			boost::filesystem::directory_iterator end_itr;
		if (boost::filesystem::is_directory (dir_))
		{
		for (boost::filesystem::directory_iterator itr (dir_); itr != end_itr; ++itr)
		{
			std::string ext = itr->path ().extension ().string ();
			if (ext.compare (".pcd") == 0)
			{
			pcd_files_.push_back (itr->path ().string ());
			pcd_paths_.push_back (itr->path ());
			}
			else
			{
			PCL_DEBUG ("[PCDVideoPlayer::selectFolderButtonPressed] : found a different file\n");
			}
		}
		}
		else
		{
		PCL_ERROR("Path is not a directory\n");
		exit(-1);
		}
		print_info ("Have pcd file list successfully\n");
		p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
		p->createViewPort (0.0, 0, 0.5, 0.5, vp_1);
		p->createViewPort (0.5, 0, 1.0, 0.5, vp_2);
		p->createViewPort (0.0, 0.5, 1.0, 1.0, vp_3);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr back_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),forth_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>),All_raws (new pcl::PointCloud<pcl::PointXYZRGB>),All_Traws (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ROI_back (new pcl::PointCloud<pcl::PointXYZRGB> ()),ROI_forth (new pcl::PointCloud<pcl::PointXYZRGB> ());
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> ROI_list,ROIT_list;
		std::vector<Eigen::Matrix4f> T_Lforth2back;
		Eigen::Vector4f ROI_backmass,ROI_forthmass;
		int  size_squences=pcd_files_.size();
		std::cout<<"Total file of squences is"<<size_squences<<endl;

		for(int i=0;i<size_squences;i++)
		{
			pcl::io::loadPCDFile (pcd_files_[i], *back_cloud);
			Eigen::Quaternionf ori(1,0,0,0);
			back_cloud->sensor_orientation_=ori;
			std::cout<<"after reading file :"<<i+1<<" "<<endl;
			*cloud_filtered=*back_cloud;
			std::cout<<"show the raw data!"<<endl;
			showCloudstmp(cloud_filtered);

			//滤波模块
			if(voxel_filter==true)
			{
			double voxel_size=0.01;
			pcl::VoxelGrid<pcl::PointXYZRGB> vg;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);
			vg.setInputCloud (back_cloud);
			vg.setLeafSize (voxel_size, voxel_size, voxel_size);
			vg.filter (*cloud_filtered1);
			*cloud_filtered=*cloud_filtered1;
			std::cout<<"show the filtered data!"<<endl;
			showCloudstmp(cloud_filtered);
			}
	  
			//平面检测及删除模块
			{
			pcl::PointIndices::Ptr tmpinliers (new pcl::PointIndices);
			double distance=0.03,degree=25,max=10000;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ncloud_ground_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
			Eigen::VectorXf coefficients;
			pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(cloud_filtered));
			model->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
			model->setEpsAngle (pcl::deg2rad (degree));
			pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
			ransac.setMaxIterations (max);
			ransac.setDistanceThreshold (distance);
			ransac.computeModel();
			ransac.getInliers(tmpinliers->indices);
			ransac.getModelCoefficients(coefficients);
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (tmpinliers);
			extract.setNegative (true);
			extract.filter (*Ncloud_ground_plane);
			*cloud_filtered = *Ncloud_ground_plane;
			std::cout<<"show the data after deleting ground plane!"<<endl;
			showCloudstmp(cloud_filtered);
		}
		if(del_plane==true)
		{
			double distance=0.02,ratio=0.8;
			int maxitter=10000;
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			pcl::PointIndices::Ptr tmpinliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());

			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (maxitter);
			seg.setDistanceThreshold (distance);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
			int nr_points = (int) cloud_filtered->points.size ();
			for (int i=0;i<1;i++)
			{
				seg.setInputCloud (cloud_filtered);
				seg.segment (*tmpinliers, *coefficients);
				std::cout<<"plane coefficients:" << *coefficients << std::endl;
				if (tmpinliers->indices.size () == 0)
				{
					std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
					break;
				}
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud (cloud_filtered);
				extract.setIndices (tmpinliers);
				extract.setNegative (false);
				extract.filter (*cloud_plane);
				std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
				extract.setNegative (true);
				extract.filter (*cloud_f);
				*cloud_filtered = *cloud_f;
			}
		}
			std::cout<<"show the data after deleting all planes!"<<endl;
			showCloudstmp(cloud_filtered);
	  
			//聚类分割模块
			double min=100,max=100000,nab=0.02;
			pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
			pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> reg;
			reg.setMinClusterSize (min);
			reg.setMaxClusterSize (max);
			reg.setSearchMethod (tree);
			reg.setClusterTolerance (nab);
			reg.setInputCloud (cloud_filtered);
			std::vector <pcl::PointIndices> clusters;
			reg.extract (clusters);
			int clusters_size=clusters.size();
			std::vector<Eigen::Vector4f> mass_centors;
			mass_centors.resize(clusters_size);
			for(int i=0; i<clusters_size;i++)
			{
				pcl::compute3DCentroid<pcl::PointXYZRGB,float>(*cloud_filtered,clusters[i],mass_centors[i]);
			}
			//运动分割模块
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr  temp_forthT (new pcl::PointCloud<pcl::PointXYZRGB>);
		double minimum_d=100000;
		int minimum_g;
		if(i==0)
		{
	
			p->registerPointPickingCallback (&pp_callback, static_cast<void*> (&cloud_filtered));
			p->addPointCloud(cloud_filtered,"cloud_filtered",vp_3);
			std::cout<<"Please press shift+left click chose one seed get the frist ROI group!"<<endl;
			p->spin();
			bool found=false;
			for(int i=0; i<clusters_size;i++)
			{
				if(std::find(clusters.at(i).indices.begin(),clusters.at(i).indices.end(),cidx)!=clusters.at(i).indices.end())
				{
					pcl::copyPointCloud(*cloud_filtered,clusters[i],*ROI_back);
					p->removePointCloud("cloud_filtered",vp_3);
					p->addPointCloud(ROI_back,"ROI_back",vp_3);
					ROI_backmass=mass_centors[i];
					found=true;
					print_info ("visualization of ROI in viewport 3,click q to continue\n");
					p->spin();
					break;
				}
			}
			p->removePointCloud("ROI_back");
			continue;
		}
		else 
		{
			for(int i=0; i<clusters_size;i++)
			{
				double temp=pcl::distances::l2(mass_centors[i],ROI_backmass);
				if(temp-minimum_d<0)
				{
					minimum_d=temp;
					minimum_g=i;
					ROI_forthmass=mass_centors[i];
				}
			}
			pcl::copyPointCloud(*cloud_filtered,clusters[minimum_g],*ROI_forth);
			showCloudstmp(ROI_forth,ROI_backmass,mass_centors[minimum_g]);
		}
			// ICP配准模块
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
			icp.setInputCloud(ROI_forth);
			icp.setInputTarget (ROI_back);
			icp.setRANSACOutlierRejectionThreshold( 0.01 ); 
			icp.setMaxCorrespondenceDistance (0.1); 
			icp.setMaximumIterations (5000); 
			icp.setTransformationEpsilon (1e-8); 
			icp.setEuclideanFitnessEpsilon (0.1); 
			pcl::PointCloud <pcl::PointXYZRGB> Final;
			icp.align(Final);
			Eigen::Matrix4f T_forth2back = icp.getFinalTransformation ();
			toFirstT*=T_forth2back;
			cout<<"toFirstT is "<<toFirstT<<endl;
			pcl::transformPointCloud(*ROI_forth,*temp_forthT,toFirstT);
			T_Lforth2back.push_back(T_forth2back);
			ROI_list.push_back(*ROI_back); 
			if(i==1)
			{
				*All_Traws+=*ROI_back;
				*All_Traws+=*temp_forthT; 
				*All_raws+=*ROI_back;    
				*All_raws+=*ROI_forth;
			}
			else if(i==size_squences-1)
			{
				ROI_list.push_back(*ROI_forth);
				*All_raws+=*ROI_forth; 
				*All_Traws+=*temp_forthT;
			}
			else
			{
		
				*All_Traws+=*temp_forthT;
				*All_raws+=*ROI_forth;
			}
			showCloudsRight(All_Traws);
			showCloudsLeft(All_raws);
			*ROI_back=*ROI_forth;	
		if(save_data==true)
		{
			std::stringstream ss;
			ss << i << ".pcd";
			pcl::io::savePCDFileASCII (ss.str(), *ROI_back);
		}
			ROI_backmass=ROI_forthmass;
		}

		tstop = (double)clock()/CLOCKS_PER_SEC;
		ttime= tstop-tstart; 
		std::cout<<"run time is"<<tstop<<"seconds"<<endl;
  
		return 0;
	}


