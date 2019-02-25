#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <string.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud(new PointCloudT);


//std::string world_frame_id;
std::string frame_id;

std::string path;

void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  *cloud = *callback_cloud;
  frame_id = cloud->header.frame_id;
  std::stringstream ss;
  std::string s1;
  ss<<ros::Time::now();
  s1=ss.str();

  // cloud saving:
  std::string file_name=path+"cloud_"+frame_id.substr(0, frame_id.length())+"_"+s1+".pcd";
  std::cout<<file_name<<std::endl;
  pcl::io::savePCDFileBinary(file_name,*cloud); 	
}

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "save_cloud");
  ros::NodeHandle nh("~");
 // std::cout << "node successfully created!" << std::endl;
	
  //Read some parameters from launch file:
  std::string pointcloud_topic = "/camera/depth_registered/points";
  path = "/home/suyang/pointtest/";
 /* nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));
  nh.param("path", path, std::string("/home/ubuntu1/pointtest/"));
  std::cout << "Read some parameters from launch file." << std::endl;*/
	
	
  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);
  std::cout<<"receive messages successfully!"<<std::endl;
	
  ros::spin();

  return 0;
}

