//  DllLoader.cpp: interface for the CDllLoader class.
//
//   Version 2.1.0.4
//
//   Copyright 2008
// 
//   Sebastian Lueth
//   MICRO-EPSILON Optronic GmbH
//   Lessingstrasse 14
//   01465 Dresden OT Langebrueck
//   Germany

#ifndef LLTLoeaderH
#define LLTLoeaderH
//------------------------------------------------------------------------------

#include <Windows.h>
#include <string>

class CDllLoader
{
public:
//==============================================================================
// construction/destruction
//==============================================================================

  CDllLoader(const char* DLLName, bool* pLoadError = NULL);
  virtual ~CDllLoader();

  void Reload(bool* pLoadError = NULL);
  
  FARPROC GetFunction(const char* FuncName);
private:

  void Load( const char* Dll, bool* pLoadError);
  void Unload();

  void HandleFatalError( const char* Dll);

  HMODULE m_hDLL;                     //pointer to the dll-module
  std::string m_DLLName;
};

//------------------------------------------------------------------------------
#endif

