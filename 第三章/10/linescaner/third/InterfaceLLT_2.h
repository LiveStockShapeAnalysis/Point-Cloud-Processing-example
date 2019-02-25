//  InterfaceLLT.h: interface for the CInterfaceLLT class.
//
//   Version 3.0.0.0
//
//   Copyright 2010
// 
//   Sebastian Lueth
//   MICRO-EPSILON Optronic GmbH
//   Lessingstrasse 14
//   01465 Dresden OT Langebrueck
//   Germany

#if !defined InterfaceLLT2H
#define InterfaceLLT2H

#include "windows.h"
#include "scanControlDataTypes.h"

// declare the old functions as deprecated
#ifdef DEPRECATED
#undef DEPRECATED
#endif

#if defined _MSC_VER
#define DEPRECATED __declspec(deprecated)
#else
#define DEPRECATED 
#endif

class CDllLoader;

//LLT-Interface class
class CInterfaceLLT
{
public:
  CInterfaceLLT(const char* pDLLName = ".\\LLT.dll", bool* pLoadError =NULL);
  ~CInterfaceLLT();
 
  class CFunctions
  {
  public:
    CFunctions(const char* pDLLName, bool* pLoadError =NULL);
    ~CFunctions();

    //Instance functions
    typedef long (__stdcall *TInterfaceCreateLLTDevice)(int iInterfaceType);
    typedef long (__stdcall *TInterfaceCreateLLTFirewire)();
    typedef long (__stdcall *TInterfaceCreateLLTSerial)();
    typedef int (__stdcall *TInterfaceGetInterfaceType)(long pLLT);
    typedef int (__stdcall *TInterfaceDelDevice)(long pLLT);

    //Connect functions
    typedef int (__stdcall *TInterfaceConnect)(long pLLT);
    typedef int (__stdcall *TInterfaceDisconnect)(long pLLT);

    //Write config functions
    typedef int  (__stdcall *TInterfaceExportLLTConfig)(long pLLT, const char *pFileName);

    //Device interface functions
    typedef int (__stdcall *TInterfaceGetDeviceInterfaces)(long pLLT, unsigned int *pInterfaces, unsigned int nSize);
    typedef int (__stdcall *TInterfaceGetDeviceInterfacesFast)(long pLLT, unsigned int *pInterfaces, unsigned int nSize);
    typedef int (__stdcall *TInterfaceSetDeviceInterface)(long pLLT, unsigned int nInterface, int nAdditional);
    typedef unsigned (__stdcall *TInterfaceGetDiscoveryBroadcastTarget)(long pLLT);
    typedef int (__stdcall *TInterfaceSetDiscoveryBroadcastTarget)(long pLLT, unsigned int nNetworkAddress, unsigned int nSubnetMask);

    //scanCONTROL indentification functions
    typedef int (__stdcall *TInterfaceGetDeviceName)(long pLLT, char *pDevName, unsigned int nDevNameSize, char* pVenName, unsigned int nVenNameSize);
    typedef int (__stdcall *TInterfaceGetLLTVersions)(long pLLT, unsigned int *pDSP, unsigned int *pFPGA1, unsigned int *pFPGA2);
    typedef int (__stdcall *TInterfaceGetLLTType)(long pLLT, TScannerType *pScannerType);
    
    //Get functions
    typedef int (__stdcall *TInterfaceGetMinMaxPacketSize)(long pLLT, unsigned long *pMinPacketSize, unsigned long *pMaxPacketSize);
    typedef int (__stdcall *TInterfaceGetResolutions)(long pLLT, DWORD *pValue, unsigned int nSize);

    typedef int (__stdcall *TInterfaceGetFeature)(long pLLT, DWORD Function, DWORD *pValue);
    typedef int (__stdcall *TInterfaceGetValue)(long pLLT, DWORD *pValue);
    typedef int (__stdcall *TInterfaceGetProfileConfig)(long pLLT, int *pValue);
    typedef int (__stdcall *TInterfaceGetProfileContainerSize)(long pLLT, unsigned int *pWidth, unsigned int *pHeight);
    typedef int (__stdcall *TInterfaceGetMaxProfileContainerSize)(long pLLT, unsigned int *pMaxWidth, unsigned int *pMaxHeight);


    //Set functions
    typedef int (__stdcall *TInterfaceSetFeature)(long pLLT, DWORD Function, DWORD Value);
    typedef int (__stdcall *TInterfaceSetValue)(long pLLT, DWORD Value);
    typedef int (__stdcall *TInterfaceSetProfileConfig)(long pLLT, int Value);
    typedef int (__stdcall *TInterfaceSetProfileContainerSize)(long pLLT, unsigned int nWidth, unsigned int nHeight);

    //Register functions 
    typedef int (__stdcall *TInterfaceRegisterCallback)(long pLLT, int CallbackType, void *pLLTProfileCallback, void *pUserData);
    typedef int (__stdcall *TInterfaceRegisterErrorMsg)(long pLLT, UINT Msg, HWND hWnd, WPARAM WParam);

    //Profile transfer functions
    typedef int (__stdcall *TInterfaceTransferProfiles)(long pLLT, int TransferProfileType, int nEnable);
    typedef int (__stdcall *TInterfaceTransferVideoStream)(long pLLT, int TransferVideoType, int nEnable, unsigned int *pWidth, unsigned int *pHeight);

    typedef int (__stdcall *TInterfaceGetProfile)(long pLLT);
    typedef int (__stdcall *TInterfaceMultiShot)(long pLLT, unsigned int nCount);
    typedef int (__stdcall *TInterfaceGetActualProfile)(long pLLT, unsigned char *pBuffer, unsigned int nBuffersize, TProfileConfig ProfileConfig, unsigned int *pLostProfiles);
    typedef int (__stdcall *TInterfaceConvertProfile2Values)(long pLLT, const unsigned char *pProfile, unsigned int nResolution, TProfileConfig ProfileConfig, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1);
    typedef int (__stdcall *TInterfaceConvertPartProfile2Values)(long pLLT, const unsigned char *pProfile, TPartialProfile *pPartialProfile, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1);
    typedef int (__stdcall *TInterfaceSetHoldBuffersForPolling)(long pLLT, unsigned int uiHoldBuffersForPolling);
    typedef int (__stdcall *TInterfaceGetHoldBuffersForPolling)(long pLLT, unsigned int *puiHoldBuffersForPolling);

    //Is functions
    typedef int (__stdcall *TInterfaceIsInterfaceType)(long pLLT, int iInterfaceType);
    typedef int (__stdcall *TInterfaceIsFirewire)(long pLLT);
    typedef int (__stdcall *TInterfaceIsSerial)(long pLLT);
    typedef int (__stdcall *TInterfaceIsTransferingProfiles)(long pLLT);

    //PartialProfile functions
    typedef int (__stdcall *TInterfaceGetPartialProfileUnitSize)(long pLLT, unsigned int *pUnitSizePoint, unsigned int *pUnitSizePointData);
    typedef int (__stdcall *TInterfaceGetPartialProfile)(long pLLT, TPartialProfile *pPartialProfile);
    typedef int (__stdcall *TInterfaceSetPartialProfile)(long pLLT, TPartialProfile *pPartialProfile);
    
    //Timestamp convert functions
    typedef void (__stdcall *TInterfaceTimestamp2CmmTriggerAndInCounter)(const unsigned char *pTimestamp, unsigned int *pInCounter, int *pCmmTrigger, int *pCmmActive, unsigned int *pCmmCount);
    typedef void (__stdcall *TInterfaceTimestamp2TimeAndCount)(const unsigned char *pTimestamp, double *pTimeShutterOpen, double *pTimeShutterClose, unsigned int *pProfileCount);

    //PostProcessing functions
    typedef int (__stdcall *TInterfaceReadPostProcessingParameter)(long pLLT, DWORD *pParameter, unsigned int nSize);
    typedef int (__stdcall *TInterfaceWritePostProcessingParameter)(long pLLT, DWORD *pParameter, unsigned int nSize);
    typedef int (__stdcall *TInterfaceConvertProfile2ModuleResult)(long pLLT, const unsigned char *pProfileBuffer, unsigned int nProfileBufferSize, unsigned char *pModuleResultBuffer, unsigned int nResultBufferSize, TPartialProfile *pPartialProfile);

    //Load/Save functions
    typedef int (__stdcall *TInterfaceSaveProfiles)(long pLLT, const char *pFilename, TFileType FileType);
    typedef int (__stdcall *TInterfaceLoadProfiles)(long pLLT, const char *pFilename, TPartialProfile *pPartialProfile, TProfileConfig *pProfileConfig, TScannerType *pScannerType, DWORD* pRearrengementProfile);
    typedef int (__stdcall *TInterfaceLoadProfilesGetPos)(long pLLT, unsigned int *pActualPosition, unsigned int *pMaxPosition);
    typedef int (__stdcall *TInterfaceLoadProfilesSetPos)(long pLLT, unsigned int nNewPosition);

    //Special CMM trigger functions
    typedef int (__stdcall *TInterfaceStartTransmissionAndCmmTrigger)(long pLLT, DWORD nCmmTrigger, TTransferProfileType TransferProfileType, unsigned int nProfilesForerun, const char *pFilename, TFileType FileType, unsigned int nTimeout);
    typedef int (__stdcall *TInterfaceStopTransmissionAndCmmTrigger)(long pLLT, int nCmmTriggerPolarity, unsigned int nTimeout);

    //Converts a error-value in a string
    typedef int (__stdcall *TInterfaceTranslateErrorValue)(long pLLT, int ErrorValue, char *pString, unsigned int nStringSize);

    //User Mode
    typedef int (__stdcall *TInterfaceGetActualUserMode)(long pLLT, unsigned int *pActualUserMode, unsigned int *pUserModeCount);
    typedef int (__stdcall *TInterfaceReadWriteUserModes)(long pLLT, int nWrite, unsigned int nUserMode);

    //Instance functions
    TInterfaceCreateLLTDevice CreateLLTDevice;
    TInterfaceCreateLLTFirewire CreateLLTFirewire;
    TInterfaceCreateLLTSerial CreateLLTSerial;
    TInterfaceGetInterfaceType GetInterfaceType;
    TInterfaceDelDevice DelDevice;

    //Connect functions
    TInterfaceConnect Connect;
    TInterfaceDisconnect Disconnect;

    //Write config functions
    TInterfaceExportLLTConfig ExportLLTConfig;

    //Device interface functions
    TInterfaceGetDeviceInterfaces GetDeviceInterfaces;
    TInterfaceGetDeviceInterfacesFast GetDeviceInterfacesFast;
    TInterfaceSetDeviceInterface SetDeviceInterface;
    TInterfaceGetDiscoveryBroadcastTarget GetDiscoveryBroadcastTarget;
    TInterfaceSetDiscoveryBroadcastTarget SetDiscoveryBroadcastTarget;

    //LLT indentification functions
    TInterfaceGetDeviceName GetDeviceName;
    TInterfaceGetLLTVersions GetLLTVersions;
    TInterfaceGetLLTType GetLLTType;

    //Get functions
    TInterfaceGetMinMaxPacketSize GetMinMaxPacketSize;
    TInterfaceGetResolutions GetResolutions;

    TInterfaceGetFeature GetFeature;
    TInterfaceGetValue GetBufferCount;    
    TInterfaceGetValue GetMainReflection;
    TInterfaceGetValue GetMaxFileSize;
    TInterfaceGetValue GetPacketSize;
    TInterfaceGetValue GetFirewireConnectionSpeed;
    TInterfaceGetValue GetResolution;
    TInterfaceGetProfileConfig GetProfileConfig;

    TInterfaceGetProfileContainerSize GetProfileContainerSize;
    TInterfaceGetMaxProfileContainerSize GetMaxProfileContainerSize;
    TInterfaceGetValue GetEthernetHeartbeatTimeout;

    //Set functions
    TInterfaceSetFeature SetFeature;
    TInterfaceSetValue SetBufferCount;
    TInterfaceSetValue SetMainReflection;
    TInterfaceSetValue SetMaxFileSize;
    TInterfaceSetValue SetPacketSize;
    TInterfaceSetValue SetFirewireConnectionSpeed;
    TInterfaceSetValue SetResolution;
    TInterfaceSetProfileConfig SetProfileConfig;
    TInterfaceSetProfileContainerSize SetProfileContainerSize;
    TInterfaceSetValue SetEthernetHeartbeatTimeout;

    //Register functions
    TInterfaceRegisterCallback RegisterCallback;
    TInterfaceRegisterErrorMsg RegisterErrorMsg;

    //Profile transfer functions
    TInterfaceGetProfile GetProfile;   
    TInterfaceTransferProfiles TransferProfiles;
    TInterfaceTransferVideoStream TransferVideoStream;
    TInterfaceMultiShot MultiShot;
    TInterfaceGetActualProfile GetActualProfile;
    TInterfaceConvertProfile2Values ConvertProfile2Values;
    TInterfaceConvertPartProfile2Values ConvertPartProfile2Values;
    TInterfaceSetHoldBuffersForPolling SetHoldBuffersForPolling;
    TInterfaceGetHoldBuffersForPolling GetHoldBuffersForPolling;

    //Is functions
    TInterfaceIsInterfaceType IsInterfaceType;
    TInterfaceIsSerial IsSerial;
    TInterfaceIsFirewire IsFirewire;
    TInterfaceIsTransferingProfiles IsTransferingProfiles;

    //PartialProfile functions
    TInterfaceGetPartialProfileUnitSize GetPartialProfileUnitSize;
    TInterfaceGetPartialProfile GetPartialProfile;
    TInterfaceSetPartialProfile SetPartialProfile;

    //Timestamp convert functions
    TInterfaceTimestamp2CmmTriggerAndInCounter Timestamp2CmmTriggerAndInCounter;
    TInterfaceTimestamp2TimeAndCount Timestamp2TimeAndCount;

    //PostProcessing functions
    TInterfaceReadPostProcessingParameter ReadPostProcessingParameter;
    TInterfaceWritePostProcessingParameter WritePostProcessingParameter;
    TInterfaceConvertProfile2ModuleResult ConvertProfile2ModuleResult;

    //Load/Save functions
    TInterfaceLoadProfiles LoadProfiles;
    TInterfaceSaveProfiles SaveProfiles;
    TInterfaceLoadProfilesGetPos LoadProfilesGetPos;
    TInterfaceLoadProfilesSetPos LoadProfilesSetPos;

    //Special CMM trigger functions
    TInterfaceStartTransmissionAndCmmTrigger StartTransmissionAndCmmTrigger;
    TInterfaceStopTransmissionAndCmmTrigger StopTransmissionAndCmmTrigger;

    //Converts a error-value in a string
    TInterfaceTranslateErrorValue TranslateErrorValue;

    //User Mode
    TInterfaceGetActualUserMode GetActualUserMode;
    TInterfaceReadWriteUserModes ReadWriteUserModes;

  protected:
    CDllLoader* m_pDllLoader;
  };

  //Instance functions
  int CreateLLTDevice(int iInterfaceType);
  DEPRECATED int CreateLLTFirewire();
  DEPRECATED int CreateLLTSerial();
  int GetInterfaceType();
  int DelDevice();

  //Connect functions
  int Connect();
  int Disconnect();

  //Write config functions
  int ExportLLTConfig(const char *pFileName);

  //Device interface functions
  int GetDeviceInterfaces(unsigned int *pInterfaces, unsigned int nSize);
  int GetDeviceInterfacesFast(unsigned int *pInterfaces, unsigned int nSize);
  int SetDeviceInterface(unsigned int nInterface, int nAdditional);
  unsigned GetDiscoveryBroadcastTarget();
  int SetDiscoveryBroadcastTarget(unsigned int nNetworkAddress, unsigned int nSubnetMask);

  //LLT indentification functions
  int GetDeviceName(char *pDevName, unsigned int nDevNameSize, char *pVenName, unsigned int nVenNameSize);
  int GetLLTVersions(unsigned int *pDSP, unsigned int *pFPGA1, unsigned int *pFPGA2);
  int GetLLTType(TScannerType *pScannerType);

  //Get functions
  int GetMinMaxPacketSize(unsigned long *pMinPacketSize, unsigned long *pMaxPacketSize);
  int GetResolutions(DWORD *pValue, unsigned int nSize);

  int GetFeature(DWORD Function, DWORD *pValue);
  int GetBufferCount(DWORD *pValue);
  int GetMainReflection(DWORD *pValue);
  int GetMaxFileSize(DWORD *pValue);
  int GetPacketSize(DWORD *pValue);
  int GetFirewireConnectionSpeed(DWORD *pValue);
  int GetProfileConfig(TProfileConfig *pValue);
  int GetResolution(DWORD *pValue);
  int GetProfileContainerSize(unsigned int *pWidth, unsigned int *pHeight);
  int GetMaxProfileContainerSize(unsigned int *pMaxWidth, unsigned int *pMaxHeight);
  int GetEthernetHeartbeatTimeout(DWORD *pValue);

  int SetFeature(DWORD Function, DWORD Value);
  int SetBufferCount(DWORD Value);
  int SetMainReflection(DWORD Value);
  int SetMaxFileSize(DWORD Value);
  int SetPacketSize(DWORD Value);
  int SetFirewireConnectionSpeed(DWORD Value);
  int SetProfileConfig(TProfileConfig Value);
  int SetResolution(DWORD Value);
  int SetProfileContainerSize(unsigned int nWidth, unsigned int nHeight);
  int SetEthernetHeartbeatTimeout(DWORD Value);

  //Register functions  
  int RegisterErrorMsg(UINT Msg, HWND hWnd, WPARAM WParam);
  int RegisterCallback(TCallbackType CallbackType, void *pLLTProfileCallback, void *pUserData);

  //Profile transfer functions
  int GetProfile();
  int TransferProfiles(int TransferProfileType, int nEnable);
  int TransferVideoStream(int TransferVideoType, int nEnable, unsigned int *pWidth, unsigned int *pHeight);
  int MultiShot(unsigned int nCount);
  int GetActualProfile(unsigned char *pBuffer, unsigned int nBuffersize, TProfileConfig ProfileConfig, unsigned int *pLostProfiles);
  int ConvertProfile2Values(const unsigned char *pProfile, unsigned int nResolution, TProfileConfig ProfileConfig, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1);
  int ConvertPartProfile2Values(const unsigned char *pProfile, TPartialProfile *pPartialProfile, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1);
  int SetHoldBuffersForPolling(unsigned int uiHoldBuffersForPolling);
  int GetHoldBuffersForPolling(unsigned int *puiHoldBuffersForPolling);

  //Is functions
  int IsInterfaceType(int iInterfaceType);
  DEPRECATED int IsFirewire();
  DEPRECATED int IsSerial();
  int IsTransferingProfiles();

  //PartialProfile functions
  int GetPartialProfileUnitSize(unsigned int *pUnitSizePoint, unsigned int *pUnitSizePointData);
  int GetPartialProfile(TPartialProfile *pPartialProfile);
  int SetPartialProfile(TPartialProfile *pPartialProfile);

  //Timestamp convert functions
  void Timestamp2CmmTriggerAndInCounter(const unsigned char *pTimestamp, unsigned int *pInCounter, int *pCmmTrigger, int *pCmmActive, unsigned int *pCmmCount);
  void Timestamp2TimeAndCount(const unsigned char *pTimestamp, double *pTimeShutterOpen, double *pTimeShutterClose, unsigned int *pProfileCount);
  
  //PostProcessing functions
  int ReadPostProcessingParameter(DWORD *pParameter, unsigned int nSize);
  int WritePostProcessingParameter(DWORD *pParameter, unsigned int nSize);
  int ConvertProfile2ModuleResult(const unsigned char *pProfileBuffer, unsigned int nProfileBufferSize, unsigned char *pModuleResultBuffer, unsigned int nResultBufferSize, TPartialProfile *pPartialProfile = NULL);
  
  //File functions
  int LoadProfiles(const char *pFilename, TPartialProfile *pPartialProfile, TProfileConfig *pProfileConfig, TScannerType *pScannerType, DWORD* pRearrengementProfile);
  int SaveProfiles(const char *pFilename, TFileType FileType);
  int LoadProfilesGetPos(unsigned int *pActualPosition, unsigned int *pMaxPosition);
  int LoadProfilesSetPos(unsigned int nNewPosition);

  //Special CMM trigger functions
  int StartTransmissionAndCmmTrigger(DWORD nCmmTrigger, TTransferProfileType TransferProfileType, unsigned int nProfilesForerun, const char *pFilename, TFileType FileType, unsigned int nTimeout);
  int StopTransmissionAndCmmTrigger(int nCmmTriggerPolarity, unsigned int nTimeout);

  //Converts a error-value in a string
  int TranslateErrorValue(int ErrorValue, char *pString, unsigned int nStringSize);

  //User Mode
  int GetActualUserMode(unsigned int *pActualUserMode, unsigned int *pUserModeCount);
  int ReadWriteUserModes(int nWrite, unsigned int nUserMode);

  CFunctions* m_pFunctions;

protected:
  long m_pLLT;
};


#endif
