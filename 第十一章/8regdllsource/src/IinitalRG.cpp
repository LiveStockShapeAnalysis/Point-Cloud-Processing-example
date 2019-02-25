#include "IinitalRG.h"

namespace RG
{
	IinitalRG::IinitalRG(float resolution=0.01,float leaf_size=1,float nor_radius=10,float key_radius=5,float search_radius=10,double epsilon_sac=100,int iter_sac=10000)
	{
	
		cout<<QThread::currentThreadId ()<<endl;
		cout<<"IinitalRG::IinitalRG()"<<endl;
		 _resolution=resolution;
		_leaf_size=leaf_size*resolution;
		_nor_radius=nor_radius*resolution;
		_key_radius=key_radius;
		_search_radius=search_radius;
		_epsilon_sac=epsilon_sac;
		_iter_sac=iter_sac;
		cloud_src.reset(new pcl::PointCloud<pcl::PointXYZ>); 
		cloud_tgt.reset(new pcl::PointCloud<pcl::PointXYZ>); 
		ds_src.reset(new pcl::PointCloud<pcl::PointXYZ>); 
		ds_tgt.reset(new pcl::PointCloud<pcl::PointXYZ>); 
		norm_src.reset(new pcl::PointCloud<pcl::Normal>);
		norm_tgt.reset(new pcl::PointCloud<pcl::Normal>);

		keypoints_src.reset(new pcl::PointCloud<pcl::PointXYZ>); 
		keypoints_tgt.reset(new pcl::PointCloud<pcl::PointXYZ>); 
	

		fpfh_src.reset(new pcl::PointCloud<pcl::FPFHSignature33>) ;
		fpfh_tgt.reset(new pcl::PointCloud<pcl::FPFHSignature33>) ;
	
		cor_all_ptr.reset(new pcl::Correspondences);
		cor_inliers_ptr.reset(new pcl::Correspondences);

		
	}
	void IinitalRG::Set_source(pcl::PointCloud<pcl::PointXYZ> cloud_src_)
	{
		cout<<QThread::currentThreadId ()<<endl;
		cout<<"IinitalRG::Set_source()"<<endl;
		*cloud_src=cloud_src_;
	}

	void IinitalRG::Set_target(pcl::PointCloud<pcl::PointXYZ> cloud_tgt_)
	{
		*cloud_tgt=cloud_tgt_;
	}

	IinitalRG::~IinitalRG()
	{
		
	}

	void IinitalRG::run()
	{
		cout<<QThread::currentThreadId ()<<endl;
		cout<<"IinitalRG::run()"<<endl;
		Downsamping(this->_leaf_size);
		Normal_estimation(this->_nor_radius);
		Keypoint_estimation(this->_key_radius);
		FPFH_feature_estimation(this->_search_radius);
		Correspondences_estimation();
		Correspondences_rejection();
		emit Initial_alignment_Done();
	}

	void IinitalRG::Downsamping(float leaf_size)
	{
		cout<<QThread::currentThreadId ()<<endl;
		cout<<"IinitalRG::Downsamping()"<<endl;
		pcl::VoxelGrid<pcl::PointXYZ> grid; 
		grid.setLeafSize (leaf_size, leaf_size, leaf_size); //adjust the resolution of downsample to keep the speed of processing
		grid.setInputCloud (cloud_src); 
		grid.filter (*ds_src); 
		grid.setInputCloud (cloud_tgt); 
		grid.filter (*ds_tgt);
		emit DownS_Done();
	}

	void IinitalRG::Normal_estimation(float nor_radius)
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>()); 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>()); 

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; 
		//Source-Cloud 
		PCL_INFO ("	Normal Estimation - Source \n");	
		ne.setInputCloud (ds_src); 
		ne.setSearchSurface (cloud_src); 
		ne.setSearchMethod (tree_src); 
		ne.setRadiusSearch (nor_radius); 
		ne.compute (*norm_src); 

		//Target-Cloud 
		PCL_INFO ("	Normal Estimation - Target \n"); 
		ne.setInputCloud (ds_tgt); 
		ne.setSearchSurface (cloud_tgt); 
		ne.setSearchMethod (tree_tgt); 
		ne.setRadiusSearch (nor_radius); 
		ne.compute (*norm_tgt); 
		emit NoramlE_Done();
	}

	void IinitalRG::Keypoint_estimation(float key_radius)
	{
		pcl::VoxelGrid<pcl::PointXYZ> grid; 
		grid.setLeafSize (key_radius,key_radius,key_radius); 
		grid.setInputCloud (ds_src); 
		grid.filter (*keypoints_src); 
		grid.setInputCloud (ds_tgt); 
		grid.filter (*keypoints_tgt);
		emit KeyP_Done();
	}

	void IinitalRG::FPFH_feature_estimation(float search_radius)
	{
		//FPFH Source 
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est_src; 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_src (new pcl::search::KdTree<pcl::PointXYZ>); 

		fpfh_est_src.setSearchSurface (ds_src);//<-------------- Use All Points for analyzing  the local structure of the cloud 
		fpfh_est_src.setInputCloud (keypoints_src); //<------------- But only compute features at the key-points 
		fpfh_est_src.setInputNormals (norm_src); 
		fpfh_est_src.setRadiusSearch (search_radius);
		fpfh_est_src.setSearchMethod(tree_fpfh_src);
		fpfh_est_src.compute (*fpfh_src); 

		//FPFH Target 
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est_tgt; 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_tgt (new pcl::search::KdTree<pcl::PointXYZ>); 
		fpfh_est_tgt.setSearchSurface (ds_tgt); 
		fpfh_est_tgt.setInputCloud (keypoints_tgt); 
		fpfh_est_tgt.setInputNormals (norm_tgt); 
		fpfh_est_tgt.setRadiusSearch (search_radius); 
		fpfh_est_tgt.setSearchMethod(tree_fpfh_tgt);
		fpfh_est_tgt.compute (*fpfh_tgt); 

		emit Feature_Done();
	}

	void IinitalRG::Correspondences_estimation()
	{
		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corEst; 
		corEst.setInputCloud (fpfh_src); 
		corEst.setInputTarget (fpfh_tgt);
		corEst.determineReciprocalCorrespondences (*cor_all_ptr);
		emit Corr_Done();
	}
	void IinitalRG::Correspondences_rejection()
	{
		//SAC 
		//double epsilon_sac = 10*voxel_re; // 10cm 
		//int iter_sac = 10000; 
		pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac; 
		//pcl::registration::corres 
		sac.setInputCloud (keypoints_src); 
		sac.setTargetCloud (keypoints_tgt); 
		sac.setInlierThreshold (_epsilon_sac); 
		sac.setMaxIterations (_iter_sac); 
		sac.setInputCorrespondences (cor_all_ptr); 
		//sac.setRefineModel(true);
		sac.getCorrespondences (*cor_inliers_ptr); 
		sac_output_transformation = sac.getBestTransformation(); 
		emit Rejection_Done();
	}
	void IinitalRG::Get_alignment_result(Eigen::Matrix4f & result) const
	{
		result=sac_output_transformation;
	}
	void IinitalRG::ColorSourceCloudDistances()
	{
		//1 apply the transformation on source point cloud get the source_inital_tcloud
		

		//2 find the correspondence for very element of source_inital_tcloud based on ceriteria


		//3 coloring the source_intial_tcloud based on the distance of correspondence


	}
}
