/*! \file LineScaner.h 
*this file declare the class which was designed for using the line scaner 
*/
#ifndef LINESCANERH
#define LINESCANERH
#include <QObject>
#include "InterfaceLLT_2.h"
#include<vector>
#include "LsLib_Export.h"
#include <Eigen/Geometry> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <boost/thread/mutex.hpp>

#define MAX_INTERFACE_COUNT    5
#define MAX_RESOULUTIONS       6
/*! \namespace LS
*LS contains all the members for line scanning
*
*/

namespace LS
{
	/*!
	* call back function provided for Line Scanner Driver
	* capturing and storing the data periodically
	*/
	void __stdcall CallBack(const unsigned char* pucData, unsigned int uiSize, void* pUserData); 
 /*! \class LineScaner
 *   Line Scanner contains all the operation  and data related with line scanning
 *
 */
	 class LsLib_EXPORT  LineScaner:public QObject
	{
		 Q_OBJECT
	public: 
/*!
*load the API dll and initalize parameter for device
*/
		 LineScaner();
		 /*!
		 *stop capturing data and diconnect device
		 */
		 ~LineScaner();
		 /*!
		 *return the number of devices connected with server
		 */
		 int NumofDevice();
		 /*!
		 *set which device is going to work default 0
		 */
		 bool SetDevice(int device_id);
		 /*!
		 *build connection with device
		 */
		 bool BuildConnection();
		 /*!
		 * get the device type store in m_tscanCONTROLType member
		 */
		 bool GetDeviceType();
		 /*!
		 *get all support resoulution store in vdwResolutions[]
		 */
		 bool GetSupportResoulutions();
		 /*!
		 *Set resoulution id defualt 0
		 */
		 bool SetResoulution(int resuolution_id);
		 /*!
		 *set filter
		 */
		 bool Setfilter(unsigned long filtering);
		 /*!
		 *set the trigger type,always internal.
		 */
		 bool SetTrigger();
		 /*!
		 *set profile type defualt PROFILE.
		 */
 bool SetProfileType(TProfileConfig type);
 /*!
 * set shutter time defualt 100
 */
		 bool SetShutterTime( unsigned int shuttertime);
		 /*!
		 *set idle time defualt 900
		 */
		 bool SetIdleTime( unsigned int idleTime);
		 /*!
		 *register callback function which will capturing data periodically
		 */
		 bool StartCapturing();
		 /*!
		 *called in callback function broadcast the new frame captured news by signal.
		 */
		 void Fire_new();
		 /*!
		 *stop calling the callback stop the transfering of data
		 */
		 bool EndCapturing();
		 /*!
		 * get how many frame are captured so far
		 */
		 void Get_num_frame(unsigned int &num_frame);
		 /*!
		 * set data state. true means data are good in sense of size
		 */
	    void SetDataState(bool test);
		/*!
		*return the resoulution(not ID)
		*/
		unsigned int GetResoulution();
		/*!
		* set the pose for transform the data in device frame to world frame.
		*must called before Get_W3D.
		*/
		void Set_Last_Pose(double x,double y,double z,double roll,double pitch,double yaw);
		/*\brief Get the lastest point cloud data in the world coordinate system
		*and sent the final data ready signal W3D_Ready(),user can be notified for further processing.
		*
		*\param[in/out] cloud store the final data.
		*/
		void Get_W3D(pcl::PointCloud<pcl::PointXYZ> &cloud);

		/**
		* convert the current frame to 3d in device frame
		*/
		void Get_L3D(pcl::PointCloud<pcl::PointXYZ> &cloud);
	protected:
	
		
		void OnError(const char* szErrorTxt, int iErrorValue);
	
	public slots:
		bool DismissConnection();
signals:
		void New_Frame_comed()const;
		void Connection_okay()const;
		void Connection_off(bool)const;
		void W3D_Ready(bool)const;
		void L3D_Ready(bool)const;

	public:
		unsigned int m_uiProfileDataSize;
		std::vector<unsigned char> vucProfileBuffer_Last;
		bool debug_verbose;

	private:
		/*!
		*convert the data frame the callback to hunman understand minimeter
		*/
		bool GetXZinMM();
		/*!
		* current frame to 3d in device frame
		*/
		void Convert_to_3DL();
		/*!
		*Get the lastest point cloud data in the world coordinate system,must called after setpose
		*/
		void Convert_to_3DW();
	private:
		boost::mutex mutex_;
		unsigned int Numer_of_new_frame;
		double _x,_y,_z,_roll,_pitch,_yaw;
		Eigen::Transform<double, 3, Eigen::Affine> last_T_W;
		std::vector<unsigned int> vuiEthernetInterfaces;
		std::vector<unsigned long> vdwResolutions;
		unsigned int uiEthernetInterfaceCount;
		unsigned int uiShutterTime;
		unsigned int uiIdleTime;
		unsigned long ulFilter;
		bool bLoadError;
		int iRetValue;
		bool bOK,Atleastone;
		bool bConnected;
		bool Capturing_state;
		bool Data_fine;
		unsigned int m_uiResolution;
		CInterfaceLLT* m_pLLT;
		TScannerType m_tscanCONTROLType;
		 std::vector<double> vdValueX_Last;
		 std::vector<double> vdValueZ_Last;
		 pcl::PointCloud<pcl::PointXYZ>::Ptr P_last_cloud_W; 
		 pcl::PointCloud<pcl::PointXYZ>::Ptr P_last_cloud_L;
	};
}

#endif