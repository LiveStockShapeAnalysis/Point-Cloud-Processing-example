// LineScaner.cpp : Defines the exported functions for the DLL application. 
//

#include "stdafx.h"
#include "LineScaner.h"
#include <iostream>
#include <conio.h>
#include <stdexcept>

using namespace std;


namespace LS
{
	LineScaner::LineScaner()
	{
		vuiEthernetInterfaces.resize(MAX_INTERFACE_COUNT);
		vdwResolutions.resize(MAX_RESOULUTIONS);
		uiEthernetInterfaceCount = 0;
		Numer_of_new_frame=0;
		uiShutterTime = 100;
		uiIdleTime = 900;
		bOK = true;
		Capturing_state=false;
		Data_fine=false;
		debug_verbose=true;
		bConnected = false;
		Atleastone = false;
		ulFilter=0;
		m_uiResolution = 0;
		P_last_cloud_W.reset(new pcl::PointCloud<pcl::PointXYZ>());
		P_last_cloud_L.reset(new pcl::PointCloud<pcl::PointXYZ>());
		m_pLLT = new CInterfaceLLT("LLT.dll", &bLoadError);
		if(bLoadError)
		{

			//Deletes the LLT-object
			delete m_pLLT;
			std::ostringstream oss; 
			oss << "Please check the LLT.DLL'version or path!"; 
			MessageBox(NULL,oss.str().c_str(), "Msg Title", MB_OK | MB_ICONQUESTION);
		}
		else
		{
			std::cout << "loading LLT.dll \n";
		}

		if(m_pLLT->m_pFunctions->CreateLLTDevice == NULL)
		{
			cout << "Please use a LLT.dll version 3.0.0.0 or higher! \n";
		}
		else
		{
			//Create a Firewire Device
			if(m_pLLT->CreateLLTDevice(INTF_TYPE_ETHERNET))
				cout << "CreateLLTDevice Ethernet OK \n";
			else
				cout << "Error during CreateLLTDevice\n";
		}
		if(!bLoadError)NumofDevice();
		if(iRetValue == ERROR_GETDEVINTERFACES_REQUEST_COUNT)
		{
			cout << "There are more or equal than " << vuiEthernetInterfaces.size() << " scanCONTROL connected \n";
			uiEthernetInterfaceCount = vuiEthernetInterfaces.size();
		}
		else if(iRetValue < 0)
		{
			cout << "A error occured during searching for connected scanCONTROL \n";
			uiEthernetInterfaceCount = 0;
		}
		else
		{
			uiEthernetInterfaceCount = iRetValue;
		}
		if(uiEthernetInterfaceCount == 0)
			cout << "There is no scanCONTROL connected \n";
		else if(uiEthernetInterfaceCount == 1)
		{
			Atleastone=true;
			cout << "There is 1 scanCONTROL connected \n";
		}
		else
		{
			Atleastone=true;
			cout << "There are " << uiEthernetInterfaceCount << " scanCONTROL's connected \n";
		}
			


	}
	LineScaner::~LineScaner()
	{
		EndCapturing();
		DismissConnection();
		if(m_pLLT!=NULL)delete m_pLLT;
	}
	int LineScaner::NumofDevice()
	{
		int Num;
		uiEthernetInterfaceCount=iRetValue=Num=m_pLLT->GetDeviceInterfacesFast(&vuiEthernetInterfaces[0], (unsigned int)vuiEthernetInterfaces.size());
		if(uiEthernetInterfaceCount>=1)this->Atleastone=true;
		return Num;
	}
	void LineScaner::OnError(const char* szErrorTxt, int iErrorValue)
	{
		char acErrorString[200];

		if(debug_verbose)cout << szErrorTxt << "\n";
		if(m_pLLT->TranslateErrorValue(iErrorValue, acErrorString, sizeof(acErrorString)) >= GENERAL_FUNCTION_OK)
			if(debug_verbose)cout << acErrorString << "\n\n";
	
	}
	bool LineScaner::SetDevice(int device_id=0)
	{
		if(Atleastone)
		{
			if(debug_verbose)cout << "\nSelect the device interface " << vuiEthernetInterfaces[device_id] << "\n";
			if((iRetValue = m_pLLT->SetDeviceInterface(vuiEthernetInterfaces[device_id], 0)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetDeviceInterface", iRetValue);
				return bOK = false;
			}
			else
				return true;
		}
		else 
		{
			return bOK = false;
			if(debug_verbose)cout << "There is no device detected" ;
		}
	}
	bool LineScaner::BuildConnection()
	{
		if(bOK)
		{
			if(debug_verbose)cout << "Connecting to scanCONTROL\n";
			if((iRetValue = m_pLLT->Connect()) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during Connect", iRetValue);
				return bOK = false;
			}
			else
			{
				emit Connection_okay();
				return bConnected = true;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
	}
	bool LineScaner::GetDeviceType()
	{
		 if(debug_verbose)  cout << "Get scanCONTROL type\n";
        if((iRetValue = m_pLLT->GetLLTType(&m_tscanCONTROLType)) < GENERAL_FUNCTION_OK)
        {
          OnError("Error during GetLLTType", iRetValue);
		 return bOK = false;
        }
		 if(iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
        {
          cout << "Can't decode scanCONTROL type. Please contact Micro-Epsilon for a newer version of the LLT.dll.\n";
        }

        if(m_tscanCONTROLType >= scanCONTROL27xx_25 && m_tscanCONTROLType <= scanCONTROL27xx_xxx)
        {
         if(debug_verbose)   cout << "The scanCONTROL is a scanCONTROL27xx\n\n";
        }
        else if(m_tscanCONTROLType >= scanCONTROL26xx_25 && m_tscanCONTROLType <= scanCONTROL26xx_xxx)
        {
          if(debug_verbose)  cout << "The scanCONTROL is a scanCONTROL26xx\n\n";
        }
        else if(m_tscanCONTROLType >= scanCONTROL29xx_25 && m_tscanCONTROLType <= scanCONTROL29xx_xxx)
        {
          if(debug_verbose)  cout << "The scanCONTROL is a scanCONTROL29xx\n\n";
        }
        else
        {
         if(debug_verbose)   cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n\n";
        }
		return true;
	}
	bool LineScaner::GetSupportResoulutions()
	{
		 if(debug_verbose) cout << "Get all possible resolutions\n";
		if((iRetValue = m_pLLT->GetResolutions(&vdwResolutions[0], vdwResolutions.size())) < GENERAL_FUNCTION_OK)
		{
			OnError("Error during GetResolutions", iRetValue);
			return bOK = false;
		}
		return true;
	}
	bool LineScaner::SetResoulution(int resuolution_id=0)
	{
		m_uiResolution = vdwResolutions[resuolution_id];
		if(bOK)
		{
			if(debug_verbose)cout << "Set resolution to " << m_uiResolution << "\n";
			if((iRetValue = m_pLLT->SetResolution(m_uiResolution)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetResolution", iRetValue);
				return bOK = false;
			}
			else 
			{
				vucProfileBuffer_Last.resize(m_uiResolution*64);
				vdValueX_Last.resize(m_uiResolution);
				vdValueZ_Last.resize(m_uiResolution);
				P_last_cloud_W->points.resize(m_uiResolution);
				P_last_cloud_L->points.resize(m_uiResolution);
				return true;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
		
	}
	unsigned int LineScaner::GetResoulution()
	{
		return m_uiResolution;
	}
	bool LineScaner::SetTrigger()
	{
		if(bOK)
		{
			if(debug_verbose)cout << "Set trigger to internal\n";
			if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_TRIGGER, 0x00000000)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", iRetValue);
				return bOK = false;
			}
			return true;
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
	}
	bool LineScaner::SetProfileType(TProfileConfig type=PROFILE)
	{
		if(bOK)
		{
			cout << "Profile config set to PROFILE\n";
			if((iRetValue = m_pLLT->SetProfileConfig(type)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetProfileConfig", iRetValue);
				bOK = false;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
		return true;
	}
	bool LineScaner::SetShutterTime( unsigned int shuttertime=100)
	{
		if(bOK)
		{
			m_pLLT->SetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA, 0xffff); // hold it
			if(debug_verbose)cout << "Set shutter time to " << shuttertime << "\n";
			if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_SHUTTERTIME, shuttertime)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetFeature(FEATURE_FUNCTION_SHUTTERTIME)", iRetValue);
				bOK = false;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
		uiShutterTime=shuttertime;
		return true;

	}
	bool LineScaner::SetIdleTime(unsigned int idleTime=900)
	{
		if(bOK)
		{
			if(debug_verbose)cout << "Set idle time to " << idleTime << "\n";
			if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_IDLETIME, idleTime)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetFeature(FEATURE_FUNCTION_IDLETIME)", iRetValue);
				bOK = false;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
		uiIdleTime=idleTime;
		return true;
	}
	bool LineScaner::DismissConnection()
	{
		if(bConnected)
		{
			if(debug_verbose)cout << "Disconnect the scanCONTROL\n";
			if((iRetValue = m_pLLT->Disconnect()) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during Disconnect", iRetValue);
				return false;
			}
			emit this->Connection_off(true);
			return true;
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
	}
	void __stdcall CallBack(const unsigned char* pucData, unsigned int uiSize, void* pUserData)
	{
		LineScaner * plinescaner =(LineScaner *)pUserData;
		boost::mutex mutex_;
		
			if(uiSize > 0)
			{
				if(plinescaner->debug_verbose)cout << "Getting new frame from callback\n";
				plinescaner->m_uiProfileDataSize = uiSize;
				if (mutex_.try_lock ())
				{
					memcpy(&plinescaner->vucProfileBuffer_Last[0], pucData, uiSize);
					mutex_.unlock ();
				}
			}
			if(plinescaner->m_uiProfileDataSize==plinescaner->GetResoulution()*64)
			{
				plinescaner->SetDataState(true);
				if(plinescaner->debug_verbose)cout <<"m_uiProfileDataSize"<< plinescaner->m_uiProfileDataSize;
			}
			else plinescaner->SetDataState(false);
	
			plinescaner->Fire_new();
		
	}
	bool LineScaner::StartCapturing()
	{
		if(debug_verbose)cout << "Register the callback\n";
		if((iRetValue = m_pLLT->RegisterCallback(STD_CALL, (void *)CallBack, this)) < GENERAL_FUNCTION_OK)
		{
			OnError("Error during RegisterCallback", iRetValue);
			return false;
		}
		if(debug_verbose)cout << "Enable the measurement\n";
		if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, true)) < GENERAL_FUNCTION_OK)
		{
			OnError("Error during TransferProfiles", iRetValue);
			return false;
		}
		return Capturing_state=true;
	}
	bool LineScaner::EndCapturing()
	{
		if(debug_verbose)cout << "Disable the measurement\n";
		if(Capturing_state==false) return true;
		if((iRetValue = m_pLLT->TransferProfiles(NORMAL_TRANSFER, false)) < GENERAL_FUNCTION_OK)
		{
			OnError("Error during TransferProfiles", iRetValue);
			return false;
		}
		Capturing_state=false;
		return true;
	}
	void LineScaner::SetDataState(bool test)
	{
		Data_fine=test;
	}
	void LineScaner::Set_Last_Pose(double x,double y,double z,double roll,double pitch,double yaw)
	{
		_x=x;
		_y=y;
		_z=z;
		_roll=roll;
		_pitch=pitch;
		_yaw=yaw;
		pcl::getTransformation<double>(x,y,z,roll,pitch,yaw,this->last_T_W);
	}
	bool LineScaner::GetXZinMM()
	{
		

		if(Data_fine==false)return false;
		if(debug_verbose)cout << "Converting of profile data from the last reflection\n";
		if (mutex_.try_lock ())
		{
			iRetValue = m_pLLT->ConvertProfile2Values(&vucProfileBuffer_Last[0], m_uiResolution, PROFILE, m_tscanCONTROLType,
			0, true, NULL, NULL, NULL, &vdValueX_Last[0], &vdValueZ_Last[0], NULL, NULL);
			mutex_.unlock ();
		}

		if(((iRetValue & CONVERT_X) == 0) || ((iRetValue & CONVERT_Z) == 0))
		{
			OnError("Error during Converting of profile data", iRetValue);
			return false;
		}
		return true;
	}
	void LineScaner::Convert_to_3DL()
	{
		int Size_i=vdValueX_Last.size();
	

			for(int i=0;i<Size_i;i++)
			{
				this->P_last_cloud_L->points[i].x=vdValueX_Last[i];
				this->P_last_cloud_L->points[i].y=0;
				this->P_last_cloud_L->points[i].z=this->vdValueZ_Last[i];
				//cout<<"this->P_last_cloud_L->points[i].x=vdValueX_Last[i]"<<this->P_last_cloud_L->points[i].x<<"="<<vdValueX_Last[i]<<endl;
				//cout<<"P_last_cloud_L->points[i].z=this->vdValueZ_Last[i]"<<P_last_cloud_L->points[i].z<<"="<<this->vdValueZ_Last[i]<<endl;

			}

		
		if(debug_verbose)cout << "Converting of last reflection to 3D in local cooridnate system\n";

	}
	void LineScaner::Convert_to_3DW()
	{
			pcl::transformPointCloud<pcl::PointXYZ,double>(*P_last_cloud_L,*P_last_cloud_W,last_T_W,true);
	}
	
	void LineScaner::Get_W3D(pcl::PointCloud<pcl::PointXYZ> &cloud)
	{
		if(this->GetXZinMM())
		{
			this->Convert_to_3DL();
			this->Convert_to_3DW();
		}
			cloud=*P_last_cloud_W;
		emit W3D_Ready(true);
	}

	void LineScaner::Get_L3D(pcl::PointCloud<pcl::PointXYZ> &cloud)
	{
		if(this->GetXZinMM())
		{
			this->Convert_to_3DL();
		}
			cloud=*(this->P_last_cloud_L);
		emit L3D_Ready(true);
	}
	void LineScaner::Fire_new()
	{
		(this->Numer_of_new_frame)++;
		emit New_Frame_comed();
	}

	void LineScaner::Get_num_frame(unsigned int &num_frame)
	{
		num_frame=this->Numer_of_new_frame;
	}

	bool LineScaner::Setfilter(unsigned long filtering)
	{
		if(bOK)
		{
			if(debug_verbose)cout << "Set filter to " << filtering << "\n";
			if((iRetValue = m_pLLT->SetFeature(FEATURE_FUNCTION_PROFILE_FILTER, filtering)) < GENERAL_FUNCTION_OK)
			{
				OnError("Error during SetFeature(FEATURE_FUNCTION_PROFILE_FILTER)", iRetValue);
				bOK = false;
			}
		}
		else
		{
			if(debug_verbose)cout << "Check the call order!\n";
			return false;
		}
		ulFilter=filtering;
		return true;
	}

}