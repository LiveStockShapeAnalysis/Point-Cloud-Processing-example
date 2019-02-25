#include <iostream>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main (int argc, char** argv)
{
    std::ifstream ifs(argv[1], std::ios::in | std::ios::binary); // 打开las文件
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs); // 读取las文件

	unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();//获取las数据点的个数

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width    = nbPoints;	//保证与las数据点的个数一致	
	cloud.height   = 1;			
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	int i=0;				
	uint16_t r1, g1, b1;	
	int r2, g2, b2;			
	uint32_t rgb;			

	while(reader.ReadNextPoint()) 
	{
		// 获取las数据的x，y，z信息
		cloud.points[i].x = (reader.GetPoint().GetX());
	    cloud.points[i].y = (reader.GetPoint().GetY());
	    cloud.points[i].z = (reader.GetPoint().GetZ());
		
		//获取las数据的r，g，b信息
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue()); 
		r2 = ceil(((float)r1/65536)*(float)256);
		g2 = ceil(((float)g1/65536)*(float)256);
		b2 = ceil(((float)b1/65536)*(float)256);
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);
		cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
					
		i++; 
	}
  
	pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);//存储为pcd类型文件
	return (0);
}