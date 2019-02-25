//  DllLoader.cpp: CDllLoader class.
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
#include "DllLoader.h"

// <summary>
// constructor of the DllLoader class
// </summary>
// <param name="char* DLLName">name of the dll</param>
// <returns></returns>

CDllLoader::CDllLoader(const char* DLLName, bool* pLoadError /*=NULL*/)
{
  //init some variables
  m_hDLL = 0;

  m_DLLName = DLLName;

  //load the dll
  Load(DLLName, pLoadError);
}

// <summary>
// destructor of the DllLoader class
// </summary>

CDllLoader::~CDllLoader()
{
  //unload the dll
  Unload();
}

// <summary>
// reloads the dll
// </summary>

void CDllLoader::Reload(bool* pLoadError /*=NULL*/)
{
  //unload the dll
  Unload();
  //load the dll
  Load(m_DLLName.c_str(), pLoadError);
}

// <summary>
// loads the dll
// </summary>
// <param name="char* Dll">name of the dll</param>

void CDllLoader::Load(const char* Dll, bool* pLoadError /*=NULL*/)
{
  //load the dll
  m_hDLL= LoadLibrary(Dll);

  //if the dll-load fails handle the error
  if( !m_hDLL && !pLoadError)
    HandleFatalError( Dll);
  else if( !m_hDLL)
    *pLoadError = true;
}

// <summary>
// unloads the dll and frees the handle
// </summary>

void CDllLoader::Unload()
{
  if( m_hDLL)
  {
    //if there is a dll free it
    FreeLibrary( m_hDLL);
    m_hDLL= 0;
  }
}

// <summary>
// gets a pointer to the requestet function
// </summary>
// <param name="char* FuncName">name of the requested function</param>
// <returns>pointer to the requested function</returns>

FARPROC CDllLoader::GetFunction(const char* FuncName)
{
  if( m_hDLL)
    {
      FARPROC FuncAdr = GetProcAddress(m_hDLL, FuncName);
      if(FuncAdr == NULL)
        {
          std::string strTemp = FuncName;
          strTemp.insert(strTemp.begin(),'_');
          return GetProcAddress(m_hDLL, strTemp.c_str());
        }
      else
        return FuncAdr;
    }
  else
    return NULL;
}

// <summary>
// handels fatal errors and aborts the application
// </summary>
// <param name="char* Dll">name of the dll</param>

void CDllLoader::HandleFatalError(const char* Dll)
{
  //create a buffer for the error-message
  LPVOID lpMsgBuf;
  //create the message text
  FormatMessage( FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR) &lpMsgBuf, 0, NULL);
  //copy the error-message in a string
  std::string Text( (LPCTSTR)lpMsgBuf);
  //free's the error-message-buffer
  LocalFree( lpMsgBuf);
  //add the dll-name to the error-string
  Text+= Dll;
  //exit the application with a error-messagebox
  FatalAppExit( 0, Text.c_str());
}

