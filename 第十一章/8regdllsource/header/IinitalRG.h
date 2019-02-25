/*! \file IinitalRG.h
*This file declare the class IinitalRG used for rough alignment
*
*/
#ifndef _IINITALRGH__ 
#define _IINITALRGH__
#include "stdafx.h"

/*! \namespace RG
*  RG contains all the members for alignment
*
*/
using namespace std;
namespace RG
{
	/*! \class IinitalRG
	* this class was used to align two point cloud roughlly
	*you need to use this class as follows if you want run this time cosumming work in the other thread
	*\code
	#include <QtCore>

	class Worker : public QObject
	{
	Q_OBJECT
	private slots:
	void onTimeout()
	{
	qDebug()<<"Worker::onTimeout get called from?: "<<QThread::currentThreadId();
	}
	};

	#include "main.moc"

	int main(int argc, char *argv[])
	{
	QCoreApplication a(argc, argv);
	qDebug()<<"From main thread: "<<QThread::currentThreadId();

	QThread t;
	QTimer timer;
	Worker worker;

	QObject::connect(&timer, SIGNAL(timeout()), &worker, SLOT(onTimeout()));
	timer.start(1000);

	timer.moveToThread(&t);
	worker.moveToThread(&t);

	t.start();

	return a.exec();
	}
	*\endcode
	*/
	 class RGlib_EXPORT  IinitalRG:public QObject
	{
		 Q_OBJECT
	public:
		/*!
		*set some parameters
		*/
		IinitalRG(float resolution,float leaf_size,float nor_radius,float key_radius,float search_radius,double epsilon_sac,int iter_sac);
		~IinitalRG();
		/*!
		*Set the source point cloud
		*/
		void Set_source(pcl::PointCloud<pcl::PointXYZ> cloud_src);
		/*!
		*Set the target point cloud
		*/
		void Set_target(pcl::PointCloud<pcl::PointXYZ> cloud_tgt);
		/*!
		* get the result of alignment
		*/
		void Get_alignment_result(Eigen::Matrix4f & result)const;

	public slots:
		/*!
		*whole flow have to be done here
		*/
		void run();

	private slots:
		/*!
		*Down samping,called in run()
		*/
		void Downsamping(float leaf_size);
		void Normal_estimation(float nor_radius);
		void Keypoint_estimation(float key_radius);
		void FPFH_feature_estimation(float search_radius);
		void Correspondences_estimation();
		void Correspondences_rejection();
		/**
		*clolor the source cloud based on distance between the source point and target point after
		* applying the transformation estimated from the initial registration
		*/
		void ColorSourceCloudDistances();

signals:
		void Rejection_Done();
		void Corr_Done();
		void Feature_Done();
		void KeyP_Done();
		void NoramlE_Done();
		void DownS_Done();
		void Initial_alignment_Done();
	private:
		float _resolution;
		float _leaf_size;
		float _nor_radius;
		float _key_radius;
		float _search_radius;
		double _epsilon_sac;
		int _iter_sac;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr ds_src; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr ds_tgt;
		pcl::PointCloud<pcl::Normal>::Ptr norm_src; 
		pcl::PointCloud<pcl::Normal>::Ptr norm_tgt;
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt; 
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src ;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt;
		boost::shared_ptr<pcl::Correspondences> cor_all_ptr; 
		boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr;
		Eigen::Matrix4f sac_output_transformation; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_initial_T;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_color_src_initial_T;


	};
}


#endif