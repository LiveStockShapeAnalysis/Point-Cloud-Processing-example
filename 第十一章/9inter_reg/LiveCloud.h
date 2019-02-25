#ifndef _LIVECLOUD__ 
#define _LIVECLOUD__
#include "stdafx_aq.h"
#include "AQlib_Export.h"

namespace AQ
{

	/**
	* \class LiveCloud
	*this class was designed for getting the live stream from device
	*/

	 class AQlib_EXPORT  LiveCloud:public QObject
	{
		 Q_OBJECT
	public:
		boost::mutex cloud_mutex_;
		boost::mutex image_mutex_;
		typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
		typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr CloudConstPtr;

		explicit LiveCloud(pcl::io::OpenNI2Grabber& grabber):grabber_ (grabber), rgb_data_ (0), rgb_data_size_ (0),started_bool_(false),save_bool_(false),cout_serials_(0),save_ply_(false)
			, image_mutex_ ()
			, image_ ()
			, depth_image_ ()
			, RGB_importer_ (vtkSmartPointer<vtkImageImport>::New ())
			, depth_importer_ (vtkSmartPointer<vtkImageImport>::New ())
			, writer_ (vtkSmartPointer<vtkTIFFWriter>::New ())
			, flipper_ (vtkSmartPointer<vtkImageFlip>::New ())
			,save_image_(false)
			,save_depth_image_(false)
			,image_serials_(0)
		{
			cloud_return.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
			cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
			RGB_importer_->SetNumberOfScalarComponents (3);
			RGB_importer_->SetDataScalarTypeToUnsignedChar ();
			depth_importer_->SetNumberOfScalarComponents (1);
			depth_importer_->SetDataScalarTypeToUnsignedShort ();
			writer_->SetCompressionToPackBits ();
			flipper_->SetFilteredAxes (1);
		}
		~LiveCloud();
		void call_back_cloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >& cloud);
		void image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image);
		/** \brief make sure using cloud_mutex_ try to lock when you copy the cloud.
		* return the pointer to const point cloud  object.
		*/
		CloudConstPtr GetCloud(); 
		boost::shared_ptr<pcl::io::openni2::Image> GetImage();
		/**
		* \brief set the dir for saving the sequence of point cloud
		*  \param[in] save_dir the dir for saving the data stream
		*  \param[in] format_ 1 pcd 2 ply more in the future
		*/
		void Set_save(std::string save_dir,int format_);
		void Set_save_RGB(std::string save_dir);
		void Set_stop_save();
 
signals:
		void NewFrame_came();
		void NewImage_came();

public slots:
	/**
	* \brief begin to get the point cloud stream from device
	*/
	void start_stream_slot(bool image_too=false);
	/**
	* \brief stop to get the point cloud stream from device
	*/
	void stop_stream_slot();
	protected:
		/**
		* \brief stop to get the point cloud stream from device
		*/
	unsigned int	Save_pointcloud_serial();
	unsigned int	Save_pointcloud_serial_ply();
	unsigned int	Save_pointcloud_serial_with_normal();
	unsigned int	Save_pointcloud_serial_ply_with_normal();
	unsigned int	Save_rgb_image_serial();
private:
	pcl::io::OpenNI2Grabber& grabber_;

	boost::signals2::connection cloud_connection;
	boost::signals2::connection image_connection;
	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::openni2::DepthImage> depth_image_;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	vtkSmartPointer<vtkImageImport> RGB_importer_, depth_importer_;
	vtkSmartPointer<vtkTIFFWriter> writer_;
	vtkSmartPointer<vtkImageFlip> flipper_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;
	bool started_bool_;/**<true indicate the stream is on*/
	bool save_bool_,image_too_;/**<true indicate saving the point cloud*/
	bool save_image_;
	bool save_depth_image_;
	bool save_ply_;/**<true indicate saving point cloud as ply format*/
	bool save_normal_;/**<true indicate saving point cloud with normal*/
	std::string Save_to_dir;
	std::string Save_image_dir_;
	std::string Save_depth_dir_;
	pcl::PCDWriter writer_pcd_;
	pcl::PLYWriter writer_ply_;
	unsigned int cout_serials_;
	unsigned int image_serials_;
	QFuture<unsigned int> result_save;
		CloudConstPtr cloud_return;
	};

}


#endif