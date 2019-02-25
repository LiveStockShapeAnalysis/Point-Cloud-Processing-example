#include "LiveCloud.h"

namespace AQ
{
	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr CloudConstPtr;
	/**
	* \class LiveCloud
	*this class was designed for getting the live stream from device
	*/
		LiveCloud::~LiveCloud()
		{

		}
		void LiveCloud::call_back_cloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >& cloud)
		{
			boost::mutex::scoped_lock lock (cloud_mutex_);
			cloud_ = cloud;
			cout_serials_++;
			if(save_bool_==true)
			{
				if(save_ply_==false)
					result_save=QtConcurrent::run(this,&LiveCloud::Save_pointcloud_serial);
					else
					result_save=QtConcurrent::run(this,&LiveCloud::Save_pointcloud_serial_ply);
			}
			NewFrame_came();
		}
		void LiveCloud::image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
		{
			boost::mutex::scoped_lock lock (image_mutex_);
			image_ = image;
			image_serials_++;
			if(save_image_==true)
			{
				result_save=QtConcurrent::run(this,&LiveCloud::Save_rgb_image_serial);
			}
			if (image->getEncoding () != pcl::io::openni2::Image::RGB)
			{
				if (rgb_data_size_ < image->getWidth () * image->getHeight ())
				{
					if (rgb_data_)
						delete [] rgb_data_;
					rgb_data_size_ = image->getWidth () * image->getHeight ();
					rgb_data_ = new unsigned char [rgb_data_size_ * 3];
				}
				image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
			}
			NewImage_came();
		}

		CloudConstPtr LiveCloud::GetCloud()
		{
			return cloud_;
		}
		boost::shared_ptr<pcl::io::openni2::Image> LiveCloud::GetImage()
		{
			return image_;
		}
		/**
		* \brief set the dir for saving the sequence of point cloud
		*/
		void LiveCloud::Set_save(std::string save_dir,int format_)
		{
			Save_to_dir=save_dir;
			if(format_==2)save_ply_=true;
			save_bool_=true;
		}
		void LiveCloud::Set_save_RGB(std::string save_dir)
		{
			Save_image_dir_=save_dir;
			save_image_=true;
		}
	/**
	* \brief begin to get the point cloud stream from device
	*/
	void LiveCloud::start_stream_slot(bool image_too)
	{
		image_too_=image_too;
		boost::function<void (const  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&) > cloud_cb = boost::bind (&LiveCloud::call_back_cloud, this, _1);
		cloud_connection = grabber_.registerCallback (cloud_cb);
		if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>()&&image_too)
		{
		
			boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&LiveCloud::image_callback, this, _1);
			image_connection = grabber_.registerCallback (image_cb);
		}
		grabber_.start ();
		started_bool_=true;
	}
	/**
	* \brief stop to get the point cloud stream from device
	*/
	void LiveCloud::stop_stream_slot()
	{
		if(started_bool_==false) return;
		grabber_.stop ();
		cloud_connection.disconnect ();
		if(image_too_)image_connection.disconnect ();
		if (rgb_data_)
			delete[] rgb_data_;
	}

	unsigned int LiveCloud::Save_pointcloud_serial()
	{
		boost::mutex::scoped_lock lock (cloud_mutex_);
		std::stringstream ss;
		ss<<Save_to_dir;
		ss<<QDir::separator().toAscii();
		ss<<std::setprecision (12) << pcl::getTime () * 100 << ".pcd";
		writer_pcd_.writeBinaryCompressed(ss.str(), *cloud_);
		return cout_serials_;
	}
	unsigned int LiveCloud::Save_rgb_image_serial()
	{
		boost::mutex::scoped_lock lock (image_mutex_);
		const void* data;
		unsigned rgb_data_size = 0;
		unsigned char* rgb_data = 0;
		if (image_)
        {
			boost::shared_ptr<pcl::io::Image> image;
			image.swap (image_);

          if (image->getEncoding() == pcl::io::openni2::Image::RGB)
          {
			  data = reinterpret_cast<const void*> (image->getData());
			  RGB_importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
			  RGB_importer_->SetDataExtentToWholeExtent ();
          }
          else
          {
            if (rgb_data_size < image->getWidth () * image->getHeight ())
            {
              rgb_data_size = image->getWidth () * image->getHeight ();
              rgb_data = new unsigned char [rgb_data_size * 3];
              RGB_importer_->SetWholeExtent (0, image->getWidth () - 1, 0, image->getHeight () - 1, 0, 0);
              RGB_importer_->SetDataExtentToWholeExtent ();
            }
            image->fillRGB (image->getWidth (), image->getHeight (), rgb_data);
            data = reinterpret_cast<const void*> (rgb_data);
          }

		  std::stringstream ss;
		  ss<<Save_image_dir_;
		  ss<<QDir::separator().toAscii();
		  ss<<std::setprecision (12) << pcl::getTime () * 100 << ".tiff";
		  RGB_importer_->SetImportVoidPointer (const_cast<void*>(data), 1);
		  RGB_importer_->Update ();
		  flipper_->SetInputConnection (RGB_importer_->GetOutputPort ());
          flipper_->Update ();
          writer_->SetFileName (ss.str ().c_str ());
          writer_->SetInputConnection (flipper_->GetOutputPort ());
          writer_->Write ();
        }
		return image_serials_;
	}
	unsigned int LiveCloud::Save_pointcloud_serial_ply()
	{
		boost::mutex::scoped_lock lock (cloud_mutex_);
		std::stringstream ss;
		ss<<Save_to_dir;
		ss<<QDir::separator().toAscii();
		ss<<std::setprecision (12) << pcl::getTime () * 100 << ".ply";
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr before_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>),after_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		std::vector<int> temp;
		Eigen::Quaternionf ori(1,0,0,0);
		*before_cloud=*cloud_;
		before_cloud->sensor_orientation_=ori;
		pcl::removeNaNFromPointCloud(*before_cloud,*after_cloud,temp);
		writer_ply_.write(ss.str(), *after_cloud,true);
		return cout_serials_;
	}
	void LiveCloud::Set_stop_save()
	{
		save_bool_=false;
		save_ply_=false;
		save_image_=false;
	}
}
