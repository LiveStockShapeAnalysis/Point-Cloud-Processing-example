//  InterfaceLLT.cpp: CInterfaceLLT class.
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

#include "stdafx.h"
#include "InterfaceLLT_2.h"
#include "DllLoader.h"

// constructor of the InterfaceLLT class
// the constructor loads the LLT.dll and request all now supported
// function-pointers so you can easy us the function of the dll
CInterfaceLLT::CFunctions::CFunctions(const char* pDLLName /*= "LLT.dll"*/, bool* pLoadError /*=NULL*/)
{
  if(pLoadError)
    *pLoadError = false;

  m_pDllLoader = new CDllLoader(pDLLName, pLoadError);

  //Instance functions
  CreateLLTDevice= (TInterfaceCreateLLTDevice) m_pDllLoader->GetFunction("s_CreateLLTDevice");
  CreateLLTFirewire= (TInterfaceCreateLLTFirewire) m_pDllLoader->GetFunction("s_CreateLLTFirewire");
  CreateLLTSerial= (TInterfaceCreateLLTSerial) m_pDllLoader->GetFunction("s_CreateLLTSerial");
  GetInterfaceType= (TInterfaceGetInterfaceType) m_pDllLoader->GetFunction("s_GetInterfaceType");
  DelDevice= (TInterfaceDelDevice) m_pDllLoader->GetFunction("s_DelDevice");

  //Connect functions
  Connect= (TInterfaceConnect) m_pDllLoader->GetFunction("s_Connect");
  Disconnect= (TInterfaceDisconnect) m_pDllLoader->GetFunction("s_Disconnect");

  //Write config functions
  ExportLLTConfig= (TInterfaceExportLLTConfig) m_pDllLoader->GetFunction("s_ExportLLTConfig");

  //Device interface functions
  GetDeviceInterfaces= (TInterfaceGetDeviceInterfaces) m_pDllLoader->GetFunction("s_GetDeviceInterfaces");
  GetDeviceInterfacesFast= (TInterfaceGetDeviceInterfacesFast) m_pDllLoader->GetFunction("s_GetDeviceInterfacesFast");
  SetDeviceInterface= (TInterfaceSetDeviceInterface) m_pDllLoader->GetFunction("s_SetDeviceInterface");
  GetDiscoveryBroadcastTarget= (TInterfaceGetDiscoveryBroadcastTarget) m_pDllLoader->GetFunction("s_GetDiscoveryBroadcastTarget");
  SetDiscoveryBroadcastTarget= (TInterfaceSetDiscoveryBroadcastTarget) m_pDllLoader->GetFunction("s_SetDiscoveryBroadcastTarget");

  //scanCONTROL indentification functions
  GetDeviceName= (TInterfaceGetDeviceName) m_pDllLoader->GetFunction("s_GetDeviceName");
  GetLLTVersions= (TInterfaceGetLLTVersions) m_pDllLoader->GetFunction("s_GetLLTVersions");
  GetLLTType= (TInterfaceGetLLTType) m_pDllLoader->GetFunction("s_GetLLTType");
 
  //Get functions
  GetMinMaxPacketSize= (TInterfaceGetMinMaxPacketSize) m_pDllLoader->GetFunction("s_GetMinMaxPacketSize");
  GetResolutions= (TInterfaceGetResolutions) m_pDllLoader->GetFunction("s_GetResolutions");

  GetFeature= (TInterfaceGetFeature) m_pDllLoader->GetFunction("s_GetFeature");
  GetBufferCount= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetBufferCount");
  GetMainReflection= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetMainReflection");
  GetMaxFileSize= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetMaxFileSize");
  GetPacketSize= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetPacketSize");
  GetFirewireConnectionSpeed= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetFirewireConnectionSpeed");
  GetProfileConfig= (TInterfaceGetProfileConfig) m_pDllLoader->GetFunction("s_GetProfileConfig");
  GetResolution= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetResolution");
  GetProfileContainerSize= (TInterfaceGetProfileContainerSize) m_pDllLoader->GetFunction("s_GetProfileContainerSize");
  GetMaxProfileContainerSize= (TInterfaceGetMaxProfileContainerSize) m_pDllLoader->GetFunction("s_GetMaxProfileContainerSize");
  GetEthernetHeartbeatTimeout= (TInterfaceGetValue) m_pDllLoader->GetFunction("s_GetEthernetHeartbeatTimeout");

  //Set functions
  SetFeature= (TInterfaceSetFeature) m_pDllLoader->GetFunction("s_SetFeature");
  SetBufferCount= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetBufferCount");
  SetMainReflection= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetMainReflection");
  SetMaxFileSize= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetMaxFileSize");
  SetPacketSize= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetPacketSize");
  SetFirewireConnectionSpeed= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetFirewireConnectionSpeed");
  SetProfileConfig= (TInterfaceSetProfileConfig) m_pDllLoader->GetFunction("s_SetProfileConfig");
  SetResolution= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetResolution");
  SetProfileContainerSize= (TInterfaceSetProfileContainerSize) m_pDllLoader->GetFunction("s_SetProfileContainerSize");
  SetEthernetHeartbeatTimeout= (TInterfaceSetValue) m_pDllLoader->GetFunction("s_SetEthernetHeartbeatTimeout");

  //Register functions  
  RegisterCallback= (TInterfaceRegisterCallback) m_pDllLoader->GetFunction("s_RegisterCallback");
  RegisterErrorMsg= (TInterfaceRegisterErrorMsg) m_pDllLoader->GetFunction("s_RegisterErrorMsg");

  //Profile transfer functions
  TransferProfiles= (TInterfaceTransferProfiles) m_pDllLoader->GetFunction("s_TransferProfiles");
  TransferVideoStream= (TInterfaceTransferVideoStream) m_pDllLoader->GetFunction("s_TransferVideoStream");
  GetProfile= (TInterfaceGetProfile) m_pDllLoader->GetFunction("s_GetProfile");
  MultiShot= (TInterfaceMultiShot) m_pDllLoader->GetFunction("s_MultiShot");
  GetActualProfile= (TInterfaceGetActualProfile) m_pDllLoader->GetFunction("s_GetActualProfile");
  ConvertProfile2Values= (TInterfaceConvertProfile2Values) m_pDllLoader->GetFunction("s_ConvertProfile2Values");
  ConvertPartProfile2Values= (TInterfaceConvertPartProfile2Values) m_pDllLoader->GetFunction("s_ConvertPartProfile2Values");
  SetHoldBuffersForPolling= (TInterfaceSetHoldBuffersForPolling) m_pDllLoader->GetFunction("s_SetHoldBuffersForPolling");
  GetHoldBuffersForPolling= (TInterfaceGetHoldBuffersForPolling) m_pDllLoader->GetFunction("s_GetHoldBuffersForPolling");

  //Is functions
  IsInterfaceType= (TInterfaceIsInterfaceType) m_pDllLoader->GetFunction("s_IsInterfaceType");
  IsFirewire= (TInterfaceIsFirewire) m_pDllLoader->GetFunction("s_IsFirewire");
  IsSerial= (TInterfaceIsSerial) m_pDllLoader->GetFunction("s_IsSerial");
  IsTransferingProfiles= (TInterfaceIsTransferingProfiles) m_pDllLoader->GetFunction("s_IsTransferingProfiles");

  //PartialProfile functions
  GetPartialProfileUnitSize= (TInterfaceGetPartialProfileUnitSize) m_pDllLoader->GetFunction("s_GetPartialProfileUnitSize");
  GetPartialProfile= (TInterfaceGetPartialProfile) m_pDllLoader->GetFunction("s_GetPartialProfile");
  SetPartialProfile= (TInterfaceSetPartialProfile) m_pDllLoader->GetFunction("s_SetPartialProfile");

  //Timestamp convert functions
  Timestamp2CmmTriggerAndInCounter= (TInterfaceTimestamp2CmmTriggerAndInCounter) m_pDllLoader->GetFunction("s_Timestamp2CmmTriggerAndInCounter");
  Timestamp2TimeAndCount= (TInterfaceTimestamp2TimeAndCount) m_pDllLoader->GetFunction("s_Timestamp2TimeAndCount");

  //PostProcessing functions
  ReadPostProcessingParameter= (TInterfaceReadPostProcessingParameter) m_pDllLoader->GetFunction("s_ReadPostProcessingParameter");
  WritePostProcessingParameter= (TInterfaceWritePostProcessingParameter) m_pDllLoader->GetFunction("s_WritePostProcessingParameter");
  ConvertProfile2ModuleResult= (TInterfaceConvertProfile2ModuleResult) m_pDllLoader->GetFunction("s_ConvertProfile2ModuleResult");

  //Load/Save functions
  LoadProfiles= (TInterfaceLoadProfiles) m_pDllLoader->GetFunction("s_LoadProfiles");
  SaveProfiles= (TInterfaceSaveProfiles) m_pDllLoader->GetFunction("s_SaveProfiles");
  LoadProfilesGetPos= (TInterfaceLoadProfilesGetPos) m_pDllLoader->GetFunction("s_LoadProfilesGetPos");
  LoadProfilesSetPos= (TInterfaceLoadProfilesSetPos) m_pDllLoader->GetFunction("s_LoadProfilesSetPos");

  //Special CMM trigger functions
  StartTransmissionAndCmmTrigger= (TInterfaceStartTransmissionAndCmmTrigger) m_pDllLoader->GetFunction("s_StartTransmissionAndCmmTrigger");
  StopTransmissionAndCmmTrigger= (TInterfaceStopTransmissionAndCmmTrigger) m_pDllLoader->GetFunction("s_StopTransmissionAndCmmTrigger");

  //Converts a error-value in a string
  TranslateErrorValue= (TInterfaceTranslateErrorValue) m_pDllLoader->GetFunction("s_TranslateErrorValue");

  //User mode
  GetActualUserMode= (TInterfaceGetActualUserMode) m_pDllLoader->GetFunction("s_GetActualUserMode");
  ReadWriteUserModes= (TInterfaceReadWriteUserModes) m_pDllLoader->GetFunction("s_ReadWriteUserModes");
}

// destructor of the InterfaceLLT class
// the destructor also unload the dll
CInterfaceLLT::CFunctions::~CFunctions()
{
  delete m_pDllLoader;
  Sleep(100);
  //the sleep in nessecary, because windows  needs a littel bit time to unload the dll
  //and if you unload and load the dll in a short time there were errors  
}

CInterfaceLLT::CInterfaceLLT(const char *pDLLName /* = "LLT.dll"*/, bool *pLoadError /*=NULL*/)
{
  m_pLLT = NULL;
  m_pFunctions = new CFunctions(pDLLName, pLoadError);
}

CInterfaceLLT::~CInterfaceLLT()
{
  if(m_pFunctions->DelDevice)
    m_pFunctions->DelDevice(m_pLLT);
  delete m_pFunctions;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//Instance functions

int CInterfaceLLT::CreateLLTDevice(int iInterfaceType)
{
  if(m_pFunctions->CreateLLTDevice != 0)
  {
    m_pLLT = m_pFunctions->CreateLLTDevice(iInterfaceType);
    if(m_pLLT != 0 || m_pLLT != 0xffffffff)
      return GENERAL_FUNCTION_OK;
  }
  return GENERAL_FUNCTION_NOT_AVAILABLE;
}

int CInterfaceLLT::CreateLLTFirewire()
{
  if(m_pFunctions->CreateLLTFirewire != 0)
  {
    m_pLLT = m_pFunctions->CreateLLTFirewire();
    if(m_pLLT != 0 || m_pLLT != 0xffffffff)
      return GENERAL_FUNCTION_OK;
  }
  return GENERAL_FUNCTION_NOT_AVAILABLE;
}

int CInterfaceLLT::CreateLLTSerial()
{
  if(m_pFunctions->CreateLLTSerial != 0)
  {
    m_pLLT = m_pFunctions->CreateLLTSerial();
    if(m_pLLT != 0 || m_pLLT != 0xffffffff)
      return GENERAL_FUNCTION_OK;
  }
  return GENERAL_FUNCTION_NOT_AVAILABLE;
}

int CInterfaceLLT::GetInterfaceType()
{
  if(m_pFunctions->GetInterfaceType != 0)
    return m_pFunctions->GetInterfaceType(m_pLLT);
  
  return GENERAL_FUNCTION_NOT_AVAILABLE;
}

int CInterfaceLLT::DelDevice()
{
  return m_pFunctions->DelDevice(m_pLLT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Connect functions

int CInterfaceLLT::Connect()
{
  return m_pFunctions->Connect(m_pLLT);
}

int CInterfaceLLT::Disconnect()
{
  return m_pFunctions->Disconnect(m_pLLT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Write config functions
int CInterfaceLLT::ExportLLTConfig(const char *pFileName)
{
  return m_pFunctions->ExportLLTConfig(m_pLLT, pFileName);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Device interface functions

int CInterfaceLLT::GetDeviceInterfaces(unsigned int *pInterfaces, unsigned int nSize)
{
  return m_pFunctions->GetDeviceInterfaces(m_pLLT, pInterfaces, nSize);
}

int CInterfaceLLT::GetDeviceInterfacesFast(unsigned int *pInterfaces, unsigned int nSize)
{
  return m_pFunctions->GetDeviceInterfacesFast(m_pLLT, pInterfaces, nSize);
}

int CInterfaceLLT::SetDeviceInterface(unsigned int nInterface, int nAdditional)
{
  return m_pFunctions->SetDeviceInterface(m_pLLT, nInterface, nAdditional);
}

unsigned CInterfaceLLT::GetDiscoveryBroadcastTarget()
{
  if(m_pFunctions->GetDiscoveryBroadcastTarget != 0)
  {
    return m_pFunctions->GetDiscoveryBroadcastTarget(m_pLLT);
  }
  return 0;
}

int CInterfaceLLT::SetDiscoveryBroadcastTarget(unsigned int nNetworkAddress, unsigned int nSubnetMask)
{
  if(m_pFunctions->SetDiscoveryBroadcastTarget != 0)
  {
    return m_pFunctions->SetDiscoveryBroadcastTarget(m_pLLT, nNetworkAddress, nSubnetMask);
  }
  return GENERAL_FUNCTION_NOT_AVAILABLE;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//LLT indentification functions

int CInterfaceLLT::GetDeviceName(char *pDevName, unsigned int nDevNameSize, char *pVenName, unsigned int nVenNameSize)
{
  return m_pFunctions->GetDeviceName(m_pLLT, pDevName, nDevNameSize, pVenName, nVenNameSize);
}

int CInterfaceLLT::GetLLTVersions(unsigned int *pDSP, unsigned int *pFPGA1, unsigned int *pFPGA2)
{
  return m_pFunctions->GetLLTVersions(m_pLLT, pDSP, pFPGA1, pFPGA2);
}

int CInterfaceLLT::GetLLTType(TScannerType *pScannerType)
{
  return m_pFunctions->GetLLTType(m_pLLT, pScannerType);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//Get functions

int CInterfaceLLT::GetMinMaxPacketSize(unsigned long *pMinPacketSize, unsigned long *pMaxPacketSize)
{
  return m_pFunctions->GetMinMaxPacketSize(m_pLLT, pMinPacketSize, pMaxPacketSize);
}

int CInterfaceLLT::GetResolutions(DWORD *pValue, unsigned int nSize)
{
  return m_pFunctions->GetResolutions(m_pLLT, pValue, nSize);
}


int CInterfaceLLT::GetFeature(DWORD Function, DWORD *pValue)
{
  return m_pFunctions->GetFeature(m_pLLT, Function, pValue);
}

int CInterfaceLLT::GetBufferCount(DWORD *pValue)
{
  return m_pFunctions->GetBufferCount(m_pLLT, pValue);
}

int CInterfaceLLT::GetMainReflection(DWORD *pValue)
{
  return m_pFunctions->GetMainReflection(m_pLLT, pValue);
}

int CInterfaceLLT::GetMaxFileSize(DWORD *pValue)
{
  return m_pFunctions->GetMaxFileSize(m_pLLT, pValue);
}

int CInterfaceLLT::GetPacketSize(DWORD *pValue)
{
  return m_pFunctions->GetPacketSize(m_pLLT, pValue);
}

int CInterfaceLLT::GetFirewireConnectionSpeed(DWORD *pValue)
{
  return m_pFunctions->GetFirewireConnectionSpeed(m_pLLT, pValue);
}

int CInterfaceLLT::GetProfileConfig(TProfileConfig *pValue)
{
  return m_pFunctions->GetProfileConfig(m_pLLT, (int*)pValue);
}

int CInterfaceLLT::GetResolution(DWORD *pValue)
{
  return m_pFunctions->GetResolution(m_pLLT, pValue);
}

int CInterfaceLLT::GetProfileContainerSize(unsigned int *pWidth, unsigned int *pHeight)
{
  return m_pFunctions->GetProfileContainerSize(m_pLLT, pWidth, pHeight);
}

int CInterfaceLLT::GetMaxProfileContainerSize(unsigned int *pMaxWidth, unsigned int *pMaxHeight)
{
  return m_pFunctions->GetMaxProfileContainerSize(m_pLLT, pMaxWidth, pMaxHeight);
}

int CInterfaceLLT::GetEthernetHeartbeatTimeout(DWORD *pValue)
{
  return m_pFunctions->GetEthernetHeartbeatTimeout(m_pLLT, pValue);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Set functions

int CInterfaceLLT::SetFeature(DWORD Function, DWORD Value)
{
  return m_pFunctions->SetFeature(m_pLLT, Function, Value);
}

int CInterfaceLLT::SetBufferCount(DWORD Value)
{
  return m_pFunctions->SetBufferCount(m_pLLT, Value);
}

int CInterfaceLLT::SetMainReflection(DWORD Value)
{
  return m_pFunctions->SetMainReflection(m_pLLT, Value);
}

int CInterfaceLLT::SetMaxFileSize(DWORD Value)
{
  return m_pFunctions->SetMaxFileSize(m_pLLT, Value);
}

int CInterfaceLLT::SetPacketSize(DWORD Value)
{
  return m_pFunctions->SetPacketSize(m_pLLT, Value);
}

int CInterfaceLLT::SetFirewireConnectionSpeed(DWORD Value)
{
  return m_pFunctions->SetFirewireConnectionSpeed(m_pLLT, Value);
}

int CInterfaceLLT::SetProfileConfig(TProfileConfig Value)
{
  return m_pFunctions->SetProfileConfig(m_pLLT, Value);
}

int CInterfaceLLT::SetResolution(DWORD Value)
{
  return m_pFunctions->SetResolution(m_pLLT, Value);
}

int CInterfaceLLT::SetProfileContainerSize(unsigned int nWidth, unsigned int nHeight)
{
  return m_pFunctions->SetProfileContainerSize(m_pLLT, nWidth, nHeight);
}

int CInterfaceLLT::SetEthernetHeartbeatTimeout(DWORD Value)
{
  return m_pFunctions->SetEthernetHeartbeatTimeout(m_pLLT, Value);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Register functions

int CInterfaceLLT::RegisterCallback(TCallbackType CallbackType, void *pLLTProfileCallback, void *pUserData)
{
  return m_pFunctions->RegisterCallback(m_pLLT, CallbackType, pLLTProfileCallback, pUserData);
}

int CInterfaceLLT::RegisterErrorMsg(UINT Msg, HWND hWnd, WPARAM WParam)
{
  return m_pFunctions->RegisterErrorMsg(m_pLLT, Msg, hWnd, WParam);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Profile transfer functions

int CInterfaceLLT::GetProfile()
{
  return m_pFunctions->GetProfile(m_pLLT);
}

int CInterfaceLLT::TransferProfiles(int TransferProfileType, int nEnable)
{
  return m_pFunctions->TransferProfiles(m_pLLT, TransferProfileType, nEnable);
}

int CInterfaceLLT::TransferVideoStream(int TransferVideoType, int nEnable, unsigned int *pWidth, unsigned int *pHeight)
{
  return m_pFunctions->TransferVideoStream(m_pLLT, TransferVideoType, nEnable, pWidth, pHeight);
}

int CInterfaceLLT::MultiShot(unsigned int nCount)
{
  return m_pFunctions->MultiShot(m_pLLT, nCount);
}

int CInterfaceLLT::GetActualProfile(unsigned char *pBuffer, unsigned int nBuffersize, TProfileConfig ProfileConfig, unsigned int *pLostProfiles)
{
  return m_pFunctions->GetActualProfile(m_pLLT, pBuffer, nBuffersize, ProfileConfig, pLostProfiles);
}

int CInterfaceLLT::ConvertProfile2Values(const unsigned char *pProfile, unsigned int nResolution, TProfileConfig ProfileConfig, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1)
{
  return m_pFunctions->ConvertProfile2Values(m_pLLT, pProfile, nResolution, ProfileConfig, ScannerType, nReflection, nConvertToMM, pWidth, pMaximum, pThreshold, pX, pZ, pM0, pM1);
}

int CInterfaceLLT::ConvertPartProfile2Values(const unsigned char *pProfile, TPartialProfile *pPartialProfile, TScannerType ScannerType, unsigned int nReflection, int nConvertToMM, unsigned short *pWidth, unsigned short *pMaximum, unsigned short *pThreshold, double *pX, double *pZ, unsigned int *pM0, unsigned int *pM1)
{
  return m_pFunctions->ConvertPartProfile2Values(m_pLLT, pProfile, pPartialProfile, ScannerType, nReflection, nConvertToMM, pWidth, pMaximum, pThreshold, pX, pZ, pM0, pM1);
}

int CInterfaceLLT::SetHoldBuffersForPolling(unsigned int uiHoldBuffersForPolling)
{
  return m_pFunctions->SetHoldBuffersForPolling(m_pLLT, uiHoldBuffersForPolling);
}

int CInterfaceLLT::GetHoldBuffersForPolling(unsigned int *puiHoldBuffersForPolling)
{
  return m_pFunctions->GetHoldBuffersForPolling(m_pLLT, puiHoldBuffersForPolling);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Is functions

int CInterfaceLLT::IsInterfaceType(int iInterfaceType)
{
  if(m_pFunctions->IsInterfaceType)
  {
    return m_pFunctions->IsInterfaceType(m_pLLT, iInterfaceType);
  }
  return 0;
}

int CInterfaceLLT::IsFirewire()
{
  return m_pFunctions->IsFirewire(m_pLLT);
}

int CInterfaceLLT::IsSerial()
{
  return m_pFunctions->IsSerial(m_pLLT);
}

int CInterfaceLLT::IsTransferingProfiles()
{
  return m_pFunctions->IsTransferingProfiles(m_pLLT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//PartialProfile functions

int CInterfaceLLT::GetPartialProfileUnitSize(unsigned int *pUnitSizePoint, unsigned int *pUnitSizePointData)
{
  return m_pFunctions->GetPartialProfileUnitSize(m_pLLT, pUnitSizePoint, pUnitSizePointData);
}

int CInterfaceLLT::GetPartialProfile(TPartialProfile *pPartialProfile)
{
  return m_pFunctions->GetPartialProfile(m_pLLT, pPartialProfile);
}

int CInterfaceLLT::SetPartialProfile(TPartialProfile *pPartialProfile)
{
  return m_pFunctions->SetPartialProfile(m_pLLT, pPartialProfile);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//Timestamp convert functions

void CInterfaceLLT::Timestamp2CmmTriggerAndInCounter(const unsigned char *pTimestamp, unsigned int *pInCounter, int *pCmmTrigger, int *pCmmActive, unsigned int *pCmmCount)
{
  m_pFunctions->Timestamp2CmmTriggerAndInCounter(pTimestamp, pInCounter, pCmmTrigger, pCmmActive, pCmmCount);
}

void CInterfaceLLT::Timestamp2TimeAndCount(const unsigned char *pTimestamp, double *pTimeShutterOpen, double *pTimeShutterClose, unsigned int *pProfileCount)
{
  m_pFunctions->Timestamp2TimeAndCount(pTimestamp, pTimeShutterOpen, pTimeShutterClose, pProfileCount);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//PostProcessing functions

int CInterfaceLLT::ReadPostProcessingParameter(DWORD* pParameter, unsigned int nSize)
{
  return m_pFunctions->ReadPostProcessingParameter(m_pLLT, pParameter, nSize);
}

int CInterfaceLLT::WritePostProcessingParameter(DWORD* pParameter, unsigned int nSize)
{
  return m_pFunctions->WritePostProcessingParameter(m_pLLT, pParameter, nSize);
}

int CInterfaceLLT::ConvertProfile2ModuleResult(const unsigned char *pProfileBuffer, unsigned int nProfileBufferSize, unsigned char *pModuleResultBuffer, unsigned int nResultBufferSize, TPartialProfile *pPartialProfile /* = NULL*/)
{
  return m_pFunctions->ConvertProfile2ModuleResult(m_pLLT, pProfileBuffer, nProfileBufferSize, pModuleResultBuffer, nResultBufferSize, pPartialProfile);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//File functions

int CInterfaceLLT::LoadProfiles(const char *pFilename, TPartialProfile *pPartialProfile, TProfileConfig *pProfileConfig, TScannerType *pScannerType, DWORD* pRearrengementProfile)
{
  return m_pFunctions->LoadProfiles(m_pLLT, pFilename, pPartialProfile, pProfileConfig, pScannerType, pRearrengementProfile);
}

int CInterfaceLLT::SaveProfiles(const char *pFilename, TFileType FileType)
{
  return m_pFunctions->SaveProfiles(m_pLLT, pFilename, FileType);
}

int CInterfaceLLT::LoadProfilesGetPos(unsigned int *pActualPosition, unsigned int *pMaxPosition)
{
  return m_pFunctions->LoadProfilesGetPos(m_pLLT, pActualPosition, pMaxPosition);
}

int CInterfaceLLT::LoadProfilesSetPos(unsigned int nNewPosition)
{
  return m_pFunctions->LoadProfilesSetPos(m_pLLT, nNewPosition);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Special CMM trigger functions

int CInterfaceLLT::StartTransmissionAndCmmTrigger(DWORD nCmmTrigger, TTransferProfileType TransferProfileType, unsigned int nProfilesForerun, const char *pFilename, TFileType FileType, unsigned int nTimeout)
{
  return m_pFunctions->StartTransmissionAndCmmTrigger(m_pLLT, nCmmTrigger, TransferProfileType, nProfilesForerun, pFilename, FileType, nTimeout);
}

int CInterfaceLLT::StopTransmissionAndCmmTrigger(int nCmmTriggerPolarity, unsigned int nTimeout)
{
  return m_pFunctions->StopTransmissionAndCmmTrigger(m_pLLT, nCmmTriggerPolarity, nTimeout);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//Converts a error-value in a string

int CInterfaceLLT::TranslateErrorValue(int ErrorValue, char *pString, unsigned int nStringSize)
{
  return m_pFunctions->TranslateErrorValue(m_pLLT, ErrorValue, pString, nStringSize);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//User Mode
int CInterfaceLLT::GetActualUserMode(unsigned int *pActualUserMode, unsigned int *pUserModeCount)
{
  return m_pFunctions->GetActualUserMode(m_pLLT, pActualUserMode, pUserModeCount);
}

int CInterfaceLLT::ReadWriteUserModes(int nWrite, unsigned int nUserMode)
{
  return m_pFunctions->ReadWriteUserModes(m_pLLT, nWrite, nUserMode);
}
