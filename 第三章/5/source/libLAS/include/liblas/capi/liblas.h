/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Main prototypes for the libLAS C API
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2008, Howard Butler
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following 
 * conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided 
 *       with the distribution.
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 ****************************************************************************/
 
#ifndef LIBLAS_H_INCLUDED
#define LIBLAS_H_INCLUDED

#define LIBLAS_C_API 1

#include "las_version.h"
#include "las_config.h"
#include <liblas/export.hpp>

typedef struct LASWriterHS *LASWriterH;
typedef struct LASReaderHS *LASReaderH;
typedef struct LASPointHS *LASPointH;
typedef struct LASHeaderHS *LASHeaderH;
typedef struct LASGuidHS *LASGuidH;
typedef struct LASVLRHS *LASVLRH;
typedef struct LASColorHS *LASColorH;
typedef struct LASSRSHS *LASSRSH;
typedef struct LASSchemaHS *LASSchemaH;


LAS_C_START


#define LAS_MODE_READ 0
#define LAS_MODE_WRITE 1
#define LAS_MODE_APPEND 2

/**
 * \todo to be documented
 */
typedef enum
{
    LE_None = 0,
    LE_Debug = 1,
    LE_Warning = 2,
    LE_Failure = 3,
    LE_Fatal = 4
} LASError;



/** Returns the version string for this library.
 *  @return the version string for this library.
*/
LAS_DLL char* LAS_GetVersion(void);
LAS_DLL char* LAS_GetFullVersion(void);

LAS_DLL int LAS_IsLibGeoTIFFEnabled(void);

LAS_DLL int LAS_IsGDALEnabled(void);

LAS_DLL int LAS_IsLibSpatialIndexEnabled(void);
/****************************************************************************/
/* Error handling                                                           */
/****************************************************************************/

/** Resets the error stack for the libLAS C API.  
*/
LAS_DLL void LASError_Reset(void);

/** Pops the top error off of the error stack for the libLAS C API.
*/
LAS_DLL void LASError_Pop(void);

/** Returns the error number of the last error on the error stack.
 *  @return the error number of the last error on the error stack.
*/
LAS_DLL LASError LASError_GetLastErrorNum(void);

/** Returns the name of the method the last error message happened in.
 *  @return the name of the method the last error message happened in.
*/
LAS_DLL char * LASError_GetLastErrorMsg(void);

/** Returns the name of the method the last error message happened in.
 *  @return the name of the method the last error message happened in.
*/
LAS_DLL char * LASError_GetLastErrorMethod(void);

/** Returns the number of error messages on the error stack.
 *  @return the number of error messages on the error stack.
*/
LAS_DLL int LASError_GetErrorCount(void);

/** Prints the last error message in the error stack to stderr.  If 
 *  there is no error on the error stack, only the message is printed.
 *  The function does not alter the error stack in any way.
 *  @param message Message to include in the stderr output
*/
LAS_DLL void LASError_Print(const char* message);

/****************************************************************************/
/* Reader operations                                                        */
/****************************************************************************/

/** Creates a LASReaderH object that can be used to read LASHeaderH and 
 *  LASPointH objects with.  The LASReaderH must not be created with a 
 *  filename that is opened for read or write by any other API functions.
 *  @return opaque pointer to a LASReaderH instance.
 *  @param filename Filename to open for read 
*/
LAS_DLL LASReaderH LASReader_Create(const char * filename);

/** Creates a LASReaderH object that can be used to read LASHeaderH and 
 *  LASPointH objects with.  The LASReaderH must not be created with a 
 *  filename that is opened for read or write by any other API functions.
 *  This function allows you to optionally override the file's header 
 *  information with your own.  
 *  @return opaque pointer to a LASReaderH instance.
 *  @param filename Filename to open for read 
 *  @param hHeader a LASHeaderH instance to override the file's header with.
 *  
*/
LAS_DLL LASReaderH LASReader_CreateWithHeader(  const char * filename, 
                                                LASHeaderH hHeader);

/** Reads the next available point on the LASReaderH instance.  If no point 
 *  is available to read, NULL is returned.  If an error happens during 
 *  the reading of the next available point, an error will be added to the 
 *  error stack and can be returned with the LASError_GetLastError* methods.
 *  @param hReader the opaque handle to the LASReaderH 
 *  @return an opaque handle to a LASPointH object, or NULL if no point is 
 *  available to read or an error occured.  Use the 
 *  LASError_GetLastError* methods to confirm the existence of an error 
 *  if NULL is returned.
*/
LAS_DLL LASPointH LASReader_GetNextPoint(const LASReaderH hReader);

/** Reads a LASPointH from the given position in the LAS file represented 
 *  by the LASReaderH instance.  If no point is available at that location, 
 *  NULL is returned.  If an error happens during the reading of the point, 
 *  an error will be added to the error stack and can be returned with the 
 *  LASError_GetLastError* methods.
 *  @param hReader the opaque handle to the LASReaderH
 *  @param position the integer position of the point in the file to read.
 *  @return an opaque handle to a LASPointH object, or NULL if no point is 
 *  available at the given location or an error occured.  Use the 
 *  LASError_GetLastError* methods to confirm the existence of an error 
 *  if NULL is returned.
*/
LAS_DLL LASPointH LASReader_GetPointAt(const LASReaderH hReader, unsigned int position);

/** Closes the file for reading operations represented by the LASReaderH instance.
 *  @param hReader the opqaue handle to the LASReaderH
*/
LAS_DLL void LASReader_Destroy(LASReaderH hReader);

/** Returns a LASHeaderH representing the header for the file
 *  @param hReader the LASReaderH instance
 *  @return a LASHeaderH representing the header for the file.  NULL is returned 
 *  in the event of an error.  Use the LASError_GetLastError* methods to check
 *  in the event of a NULL return.
*/
LAS_DLL LASHeaderH LASReader_GetHeader(const LASReaderH hReader);

LAS_DLL void LASReader_SetHeader(  LASReaderH hReader, const LASHeaderH hHeader);

LAS_DLL LASError LASReader_SetSRS(LASReaderH hReader, const LASSRSH hSRS);
LAS_DLL LASError LASReader_SetInputSRS(LASReaderH hReader, const LASSRSH hSRS);
LAS_DLL LASError LASReader_SetOutputSRS(LASReaderH hReader, const LASSRSH hSRS);

/** Seeks to the specified point for the next LASReader_GetNextPoint
 *  operation to start from.  If an error is returned, the seek failed
 *  for some reason.
 *  @param hReader the LASReaderH instance
 *  @return a LASError defaulting to LE_None upon success.  
*/
LAS_DLL LASError LASReader_Seek(LASReaderH hReader, unsigned int position);

LAS_DLL char* LASReader_GetSummaryXML(const LASReaderH hReader);

/****************************************************************************/
/* Point operations                                                         */
/****************************************************************************/

/** Returns the X value for the point.  This value is scaled by any header 
 *  information that is present for the point.  Use GetRawX if you want unscaled 
 *  data.  
 *  header values after the value is read.
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the X value for the LASPointH
*/
LAS_DLL double LASPoint_GetX(const LASPointH hPoint);

/** Sets the X value for the point.  This value is scaled by any header 
 *  information that is present for the point.  Use SetRawX if you want unscaled 
 *  data. 
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the X value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetX(LASPointH hPoint, double value);

/** Returns the raw X value for the point.  This value is not scaled or offset
 *  by any header values and stands on its own.  If you need points to have 
 *  a scale and/or offset applied, this must be done in conjunction with the 
 *  header values after the value is read.
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the raw(unscaled) X value for the LASPointH
*/
LAS_DLL long LASPoint_GetRawX(const LASPointH hPoint);

/** Sets the raw X value for the point.  This value will be scaled and offset 
 *  by any header values to get interpreted values (double GetX())
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the raw X value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetRawX(LASPointH hPoint, long value);

/** Gets the Y value for the point.  This value is scaled by any header 
 *  information that is present for the point.  Use SetRawY if you want unscaled 
 *  data. 
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the Y value for the LASPointH
*/
LAS_DLL double LASPoint_GetY(const LASPointH hPoint);

/** Sets the Y value for the point.  This value must be scaled or offset 
 *  by any header values before being set.
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the Y value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetY(LASPointH hPoint, double value);

/** Gets the raw Y value for the point.  This value will be scaled and offset 
 *  by any header values to get interpreted values (double GetY())
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the raw(unscaled) Y value for the LASPointH
*/
LAS_DLL long LASPoint_GetRawY(const LASPointH hPoint);

/** Sets the raw Y value for the point.  This value will be scaled and offset 
 *  by any header values to get interpreted values (double GetY())
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the raw Y value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetRawY(LASPointH hPoint, long value);

/** Gets the Z value for the point.  This value is scaled by any header 
 *  information that is present for the point.  Use SetRawZ if you want unscaled 
 *  data. 
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the Z value for the LASPointH
*/
LAS_DLL double LASPoint_GetZ(const LASPointH hPoint);

/** Sets the Z value for the point.  This value must be scaled or offset 
 *  by any header values before being set.
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the Z value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetZ(LASPointH hPoint, double value);

/** Gets the raw Z value for the point.  This value will be scaled and offset 
 *  by any header values to get interpreted values (double GetZ())
 *  @param hPoint the opaque pointer to the LASPointH instance  
 *  @return the raw(unscaled) Z value for the LASPointH
*/
LAS_DLL long LASPoint_GetRawZ(const LASPointH hPoint);

/** Sets the raw Z value for the point.  This value will be scaled and offset 
 *  by any header values to get interpreted values (double GetZ())
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the double value to set for the raw Z value of the point
 *  @return an error number if an error occured during the setting of the point.
*/
LAS_DLL LASError LASPoint_SetRawZ(LASPointH hPoint, long value);


/** Returns the intensity value for the point.  This value is the pulse return 
 *  magnitude, it is optional, and it is LiDAR system specific.
 *  @return the intensity value for the point.
*/
LAS_DLL unsigned short LASPoint_GetIntensity(const LASPointH hPoint);

/** Sets the intensity value for the point.
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param value the value to set the intensity to
 *  @return an error number if an error occured.
*/
LAS_DLL LASError LASPoint_SetIntensity(LASPointH hPoint, unsigned short value);

/** Returns the return number for the point.  The return number is "the pulse
 *  return number for a given output pulse."  The first return number starts with
 *  the value 1.
 *  @param hPoint LASPointH instance
 *  @return a return number, valid from 1-6, for the point.  Use the LASError 
 *  methods to determine if an error occurred during this operation if 0 
 *  is returned.
*/
LAS_DLL unsigned short LASPoint_GetReturnNumber(const LASPointH hPoint);

/** Sets the return number for the point.  Valid values are from 1-6.
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the return number
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetReturnNumber(LASPointH hPoint, unsigned short value);

/** Returns the total number of returns for a given pulse.
 *  @param hPoint LASPointH instance
 *  @return total number of returns for this pulse.
*/
LAS_DLL unsigned short LASPoint_GetNumberOfReturns(const LASPointH hPoint);

/** Sets the number of returns for the point.  Valid values are from 1-5.
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the number of returns
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetNumberOfReturns(LASPointH hPoint, unsigned short value);

/** Returns the scan direction for a given pulse.
 *  @param hPoint LASPointH instance
 *  @return the scan direction for a given pulse.
*/
LAS_DLL unsigned short LASPoint_GetScanDirection(const LASPointH hPoint);

/** Sets the scan direction for a given pulse.  Valid values are 0 or 1, with 
 *  1 being a positive scan direction and 0 being a negative scan direction.
 *  @param hPoint LASPointH instance
 *  @param value the value to set for scan direction
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetScanDirection(LASPointH hPoint, unsigned short value);

/** Returns whether or not a given pulse is an edge point
 *  @param hPoint LASPointH instance
 *  @return whether or not a given pulse is an edge point.
*/
LAS_DLL unsigned short LASPoint_GetFlightLineEdge(const LASPointH hPoint);

/** Sets the edge marker for a given pulse.  Valid values are 0 or 1, with 
 *  1 being an edge point and 0 being interior.
 *  @param hPoint LASPointH instance
 *  @param value the value to set for flightline edge
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetFlightLineEdge(LASPointH hPoint, unsigned short value);

/** Returns all of the scan flags for the point -- Return number, number of 
 *  returns, flightline edge, scan direction, and scan angle rank.
 *  @param hPoint LASPointH instance
 *  @return all of the scan flags for the point
*/
LAS_DLL unsigned char LASPoint_GetScanFlags(const LASPointH hPoint);

/** Sets all of the scan flags for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the flags
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetScanFlags(LASPointH hPoint, unsigned char value);

/** Returns the classification for the point
 *  @param hPoint LASPointH instance
 *  @return the classification for the point
*/
LAS_DLL unsigned char LASPoint_GetClassification(const LASPointH hPoint);

/** Sets the classification for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the classification
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetClassification(LASPointH hPoint, unsigned char value);

/** Returns the time for the point
 *  @param hPoint LASPointH instance
 *  @return the time for the point
*/
LAS_DLL double LASPoint_GetTime(const LASPointH hPoint);

/** Sets the time for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the time
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetTime(LASPointH hPoint, double value);

/** Returns the scan angle for the point
 *  @param hPoint LASPointH instance
 *  @return the scan angle for the point
*/
LAS_DLL char LASPoint_GetScanAngleRank(const LASPointH hPoint);

/** Sets the scan angle for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the scan angle
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetScanAngleRank(LASPointH hPoint, char value);

/** Sets the point source id for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the point source id
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetPointSourceId(LASPointH hPoint, unsigned short value);

/** Returns the point source id for the point
 *  @param hPoint LASPointH instance
 *  @return the scan angle for the point
*/
LAS_DLL unsigned short LASPoint_GetPointSourceId(LASPointH hPoint);

/** Returns the arbitrary user data for the point
 *  @param hPoint LASPointH instance
 *  @return the arbitrary user data for the point
*/
LAS_DLL unsigned char LASPoint_GetUserData(const LASPointH hPoint);

/** Sets the arbitrary user data for the point.  No validation is done. 
 *  @param hPoint LASPointH instance
 *  @param value the value to set for the arbitrary user data
 *  @return LASError value determine success or failure.
*/
LAS_DLL LASError LASPoint_SetUserData(LASPointH hPoint, unsigned char value);

/** Returns a bitfield representing the validity of various members
 *  enum DataMemberFlag
 {
     eReturnNumber = 1,
     eNumberOfReturns = 2,
     eScanDirection = 4,
     eFlightLineEdge = 8,
     eClassification = 16,
     eScanAngleRank = 32,
     eTime = 64
 };
 *  @param hPoint LASPointH instance
 *  @return bitfield representing the validity of various members.
*/
LAS_DLL int LASPoint_Validate(LASPointH hPoint);

/** Returns a boolean whether or not the point is valid
 *  @param hPoint LASPointH instance
 *  @return a boolean (1 or 0) whether or not the point is valid.
*/
LAS_DLL int LASPoint_IsValid(LASPointH hPoint);

/** Creates a new empty LASPointH instance 
 *  @return LASPointH instance.  If the value is NULL use the 
 *  LASError_GetLastError* methods to determine the problem
*/
LAS_DLL LASPointH LASPoint_Create(void);

/** Creates a copy of a LASPointH instance
 *  @param hPoint the LASPointH instance to copy 
 *  @return new LASPointH instance.  If the value is NULL use the 
 *  LASError_GetLastError* methods to determine the problem
*/
LAS_DLL LASPointH LASPoint_Copy(const LASPointH hPoint);

/** Destroys/deletes a LASPointH instance
*/
LAS_DLL void LASPoint_Destroy(LASPointH hPoint);

/** Returns a LASHeaderH representing the header for the point
 *  @param hPoint the LASPointH instance
 *  @return a LASHeaderH representing the header for the point
*/ 
LAS_DLL LASHeaderH LASPoint_GetHeader(const LASPointH hPoint);

LAS_DLL void LASPoint_SetHeader( LASPointH hPoint, const LASHeaderH hHeader);

/** Gets the data stream for the VLR as an array of bytes.  The length of this 
 *  array should be the same as LASVLR_GetRecordLength.  You must allocate it on 
 *  the heap and you are responsible for its destruction.
 *  @param hPoint the LASPointH instance
 *  @param data a pointer to your array where you want the data copied
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASPoint_GetData(const LASPointH hPoint, unsigned char* data);

/** Sets the data stream for the Point as an array of bytes.  The length of this 
 *  array should be the same as LASPoint_GetHeader(LASHeader_GetDataRecordLength()).  The data are copied into 
 *  the Point .
 *  @param hPoint the LASPointH instance
 *  @param data a pointer to your array.  It must be LASPoint_GetHeader(LASHeader_GetDataRecordLength()) in size
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASPoint_SetData(LASPointH hPoint, unsigned char* data);


/** Returns an XMLized representation of the point
 *  @param hPoint the LASPointH instance
*/
LAS_DLL char* LASPoint_GetXML(const LASPointH hPoint);

/****************************************************************************/
/* Header operations                                                        */
/****************************************************************************/

/** Copies a LASHeaderH instance
 *  @param hHeader the LASHeaderH to copy
 *  @return a LASHeaderH instance or NULL on error
*/
LAS_DLL LASHeaderH LASHeader_Copy(const LASHeaderH hHeader);

/** Creates an empty LASHeaderH with default values
*/
LAS_DLL LASHeaderH LASHeader_Create(void);

/** Destroys/deletes a LASHeader instance
*/
LAS_DLL void LASHeader_Destroy(LASHeaderH hHeader);

/** Returns the file signature the the file.  This should always be 'LASF'
 *  @param hHeader LASHeaderH instance
 *  @return the file signature the the file.  This should always be 'LASF'
*/
LAS_DLL char *LASHeader_GetFileSignature(const LASHeaderH hHeader);

/** Returns the file source id for the file.  It is a number from 1-65535
 *  @param hHeader LASHeaderH instance
 *  @return the file source id for the file.
*/
LAS_DLL unsigned short LASHeader_GetFileSourceId(const LASHeaderH hHeader);

/** Sets the FileSource ID value for the header.  By default, this value is "0" if it 
 *  is not explicitly set.  See the LAS specification for details on what this
 *  value should logically be set to.
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set as the FileSource ID value for the header
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetFileSourceId(LASHeaderH hHeader, unsigned short value);

/** Returns the project id for the header as a GUID string
 *  @return the project id for the header as a GUID string
*/
LAS_DLL char *LASHeader_GetProjectId(const LASHeaderH hHeader);

/** Sets the project id/GUID for the header
 *  @param hHeader LASHeaderH instance
 *  @param value character value GUID to set the header value to
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetProjectId(LASHeaderH hHeader, const char* value);

/** Sets the project id/GUID for the header
 *  @param hHeader LASHeaderH instance
 *  @param hId LASGuidH instance to set the GUID for the header to
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetGUID(LASHeaderH hHeader, LASGuidH hId);

/** Returns the major version number for the header.  This value is expected 
 *  to always be 1.
 *  @param hHeader LASHeaderH instance
 *  @return major version number for the header.
*/
LAS_DLL unsigned char LASHeader_GetVersionMajor(const LASHeaderH hHeader);

/** Sets the major version number for the header.  All values other than 1 
 *  are invalid.
 *  @param hHeader LASHeaderH instance
 *  @param value integer value to set the major version to (only the value 1 is valid)
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetVersionMajor(LASHeaderH hHeader, unsigned char value);

/** Returns the min version number for the header.  This value is expected 
 *  to be 1 or 0 representing LAS 1.1 or LAS 1.0
 *  @param hHeader LASHeaderH instance
 *  @return minor version number for the header.
*/
LAS_DLL unsigned char LASHeader_GetVersionMinor(const LASHeaderH hHeader);

/** Sets the minor version number for the header.  All values other than 1 or 0 
 *  are invalid.
 *  @param hHeader LASHeaderH instance
 *  @param value integer value to set the minor version to (only the values 1 or 0 are valid)
 *  @return LASError enum
 *
 *  @todo TODO: Maybe this should return fatal error if version out of range -- hobu
*/
LAS_DLL LASError LASHeader_SetVersionMinor(LASHeaderH hHeader, unsigned char value);

/** Returns the System ID for the header.  The caller assumes ownership of the returned string
 *  @return the system id for the header as a character array
*/
LAS_DLL char *LASHeader_GetSystemId(const LASHeaderH hHeader);

/** Sets the System ID for the header.  By default, this value is "libLAS" if it 
 *  is not explicitly set.  See the LAS specification for details on what this
 *  value should logically be set to.
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set as the System ID for the header
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetSystemId(LASHeaderH hHeader, const char* value);

/** Returns the Software ID for the header.  The caller assumes ownership of the returned string
 *  @return the software id for the header as a character array
*/
LAS_DLL char *LASHeader_GetSoftwareId(const LASHeaderH hHeader);

/** Sets the Software ID for the header.  By default, this value is "libLAS 1.0" if it 
 *  is not explicitly set.  See the LAS specification for details on what this
 *  value should logically be set to.
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set as the Software ID for the header
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetSoftwareId(LASHeaderH hHeader, const char* value);

/** Returns the reserved value for the header.  This should aways be 0.
 *  @return the reserved value for the header.
*/
LAS_DLL unsigned short LASHeader_GetReserved(const LASHeaderH hHeader);

/** Sets the Reserved value for the header.  By default, this value is "0" if it 
 *  is not explicitly set.  See the LAS specification for details on what this
 *  value should logically be set to.
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set as the reserved value for the header
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetReserved(LASHeaderH hHeader, unsigned short value);

/** Returns the file creation day of the year.  The values start from 1, being January 1st, 
 *  and end at 365 or 366 being December 31st, depending on leap year.
 *  @return the day of the year as an integer starting from 1 for the file creation.
*/
LAS_DLL unsigned short LASHeader_GetCreationDOY(const LASHeaderH hHeader);

/** Sets the file creation day of the year.  The values start from 1, being January 1st.  No
 *  date validation is done
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set as the creation day
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetCreationDOY(LASHeaderH hHeader, unsigned short value);

/** Returns the file creation year.  This is a four digit number representing the 
 *  year for the file, ie 2003, 2008, etc.
 *  @return the creation year for the file or 0 if none is set
*/
LAS_DLL unsigned short LASHeader_GetCreationYear(const LASHeaderH hHeader);

/** Sets the file creation year.  This should be a four digit number representing
 *  the year for the file, ie 2003, 2008, etc.  No validation on the value is done
 *  @param hHeader LASHeaderH instance
 *  @param value the value to set for the creation year
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetCreationYear(LASHeaderH hHeader, unsigned short value);

/** Returns the size of the header for the file in bytes.
 *  @return the size of the header for the file in bytes.
*/
LAS_DLL unsigned short LASHeader_GetHeaderSize(const LASHeaderH hHeader);

/** Returns the byte offset to the start of actual point data for the file
 *  @param hHeader LASHeaderH instance
 *  @return the type offset to the start of actual point data for the file
*/
LAS_DLL unsigned int LASHeader_GetDataOffset(const LASHeaderH hHeader);

/** Sets the location in number of bytes to start writing point data.  Any
 *  space between the end of the LASVLRHs and this value will be written with 0's.
 *  @param hHeader LASHeaderH instance
 *  @param value the long integer to set for byte location determining the end of the header
 *  @return LASError enum
*/
 LAS_DLL LASError LASHeader_SetDataOffset(const LASHeaderH hHeader, unsigned int value);

/** Returns the number of bytes between the end of the VLRs on the header to the data offset
 *  @param hHeader LASHeaderH instance
 *  @return the number of bytes between the end of the VLRs on the header to the data offset
*/
LAS_DLL unsigned int LASHeader_GetHeaderPadding(const LASHeaderH hHeader);

/** Sets the number of bytes between the end of the VLRs on the header to the data offset
 *  @param hHeader LASHeaderH instance
 *  @param value the long integer to set for the number of bytes between the end of the VLRs and the data offset
 *  @return LASError enum
*/
 LAS_DLL LASError LASHeader_SetHeaderPadding(const LASHeaderH hHeader, unsigned int value);


/** Returns the number of variable length records in the header
 *  @param hHeader LASHeaderH instance
 *  @return the number of variable length records in the header
*/
LAS_DLL unsigned int LASHeader_GetRecordsCount(const LASHeaderH hHeader);

/** Returns the record length for the points based on their data format id in bytes
 *  @param hHeader LASHeaderH instance
 *  @return the record length for the points based on their data format id in bytes
*/
LAS_DLL unsigned short LASHeader_GetDataRecordLength(const LASHeaderH hHeader);

/** Explicitly set the record length for the file.  If you set the DataFormatId,
 *  default values will be set for you.
 *  @param hHeader LASHeaderH instance
 *  @param value the value for the data record length (in bytes).
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetDataRecordLength(const LASHeaderH hHeader, unsigned short value);

/** Returns the data format id.  If this value is 1, the point data have time values
 *  associated with them.  If it is 0, the point data do not have time values.  
 *  @param hHeader LASHeaderH instance
 *  @return the data format id for the file.
*/
LAS_DLL unsigned char LASHeader_GetDataFormatId(const LASHeaderH hHeader);

/** Sets the data format id for the file.  The value should be 1 or 0, with 1 being
 *  points that contain time values and 0 being points that do not.
 *  @param hHeader LASHeaderH instance
 *  @param value the value for the data format id, 1 or 0 are valid values.
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetDataFormatId(const LASHeaderH hHeader, unsigned char value);

/** Returns the number of point records in the file.  This value may not reflect the actual 
 *  number of point records in the file.
 *  @param hHeader LASHeaderH instance
 *  @return the number of point records in the file
*/
LAS_DLL unsigned int LASHeader_GetPointRecordsCount(const LASHeaderH hHeader);

/** Sets the number of point records for the file.
 *  @param hHeader LASHeaderH instance
 *  @param value the long integer to set for the number of point records in the file
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetPointRecordsCount(const LASHeaderH hHeader, unsigned int value);

/** Returns the number of point records by return.
 *  @param hHeader LASHeaderH instance
 *  @param index the return number to fetch the count for
 *  @return the number of point records for a given return
*/
LAS_DLL unsigned int LASHeader_GetPointRecordsByReturnCount(const LASHeaderH hHeader, int index);

/** Sets the number of point records for a given return
 *  @param hHeader LASHeaderH instance
 *  @param index the return number to set the count for
 *  @param value the number of point records for the return 
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetPointRecordsByReturnCount(const LASHeaderH hHeader, int index, unsigned int value);

/** Return the X scale factor
 *  @param hHeader LASHeaderH instance
 *  @return the X scale factor
*/
LAS_DLL double LASHeader_GetScaleX(const LASHeaderH hHeader);

/** Return the Y scale factor
 *  @param hHeader LASHeaderH instance
 *  @return the Y scale factor
*/
LAS_DLL double LASHeader_GetScaleY(const LASHeaderH hHeader);

/** Return the Z scale factor
 *  @param hHeader LASHeaderH instance
 *  @return the Z scale factor
*/
LAS_DLL double LASHeader_GetScaleZ(const LASHeaderH hHeader);

/** Sets the scale factors
 *  @param hHeader LASHeaderH instance
 *  @param x the x scale factor
 *  @param y the y scale factor
 *  @param z the z scale factor
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetScale(LASHeaderH hHeader, double x, double y, double z);

/** Return the X offset
 *  @param hHeader LASHeaderH instance
 *  @return the X offset
*/
LAS_DLL double LASHeader_GetOffsetX(const LASHeaderH hHeader);

/** Return the Y offset
 *  @param hHeader LASHeaderH instance
 *  @return the Y offset
*/
LAS_DLL double LASHeader_GetOffsetY(const LASHeaderH hHeader);

/** Return the Z offset
 *  @param hHeader LASHeaderH instance
 *  @return the Z offset
*/
LAS_DLL double LASHeader_GetOffsetZ(const LASHeaderH hHeader);

/** Sets the offset values
 *  @param hHeader LASHeaderH instance
 *  @param x the x offset
 *  @param y the y offset
 *  @param z the z offset
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetOffset(LASHeaderH hHeader, double x, double y, double z);

/** Return the minimum x value
 *  @param hHeader LASHeaderH instance
 *  @return the minimum x value
*/
LAS_DLL double LASHeader_GetMinX(const LASHeaderH hHeader);

/** Return the minimum y value
 *  @param hHeader LASHeaderH instance
 *  @return the minimum y value
*/
LAS_DLL double LASHeader_GetMinY(const LASHeaderH hHeader);

/** Return the minimum z value
 *  @param hHeader LASHeaderH instance
 *  @return the minimum z value
*/
LAS_DLL double LASHeader_GetMinZ(const LASHeaderH hHeader);

/** Sets the minimum values
 *  @param hHeader LASHeaderH instance
 *  @param x the x minimum
 *  @param y the y minimum
 *  @param z the z minimum
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetMin(LASHeaderH hHeader, double x, double y, double z);

/** Return the maximum x value
 *  @param hHeader LASHeaderH instance
 *  @return the maximum x value
*/
LAS_DLL double LASHeader_GetMaxX(const LASHeaderH hHeader);

/** Return the maximum y value
 *  @param hHeader LASHeaderH instance
 *  @return the maximum y value
*/
LAS_DLL double LASHeader_GetMaxY(const LASHeaderH hHeader);

/** Return the maximum z value
 *  @param hHeader LASHeaderH instance
 *  @return the maximum z value
*/
LAS_DLL double LASHeader_GetMaxZ(const LASHeaderH hHeader);

/** Sets the maximum values
 *  @param hHeader LASHeaderH instance
 *  @param x the x maximum
 *  @param y the y maximum
 *  @param z the z maximum
 *  @return LASError enum
*/
LAS_DLL LASError LASHeader_SetMax(LASHeaderH hHeader, double x, double y, double z);

/** Returns the VLR record for the given index.  Use LASHeader_GetRecordsCount to 
 *  determine the number of VLR records available on the header.
 *  @param hHeader the LASHeaderH instance
 *  @param i the index starting from 0 of the VLR to fetch
 *  @return LASVLRH instance that models the Variable Length Record
*/
LAS_DLL LASVLRH LASHeader_GetVLR(const LASHeaderH hHeader, unsigned int i);

/** Deletes a VLR record from the header for the given index.
 *  @param hHeader the LASHeaderH instance
 *  @param index the index starting from 0 of the VLR to delete
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASHeader_DeleteVLR(LASHeaderH hHeader, unsigned int index);

/** Adds a VLR record to the header. 
 *  @param hHeader the LASHeaderH instance
 *  @param hVLR the VLR to add to the header
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASHeader_AddVLR(LASHeaderH hHeader, const LASVLRH hVLR);

/** Returns an XMLized representation of the header
 *  @param hHeader the LASHeader instance
*/
LAS_DLL char* LASHeader_GetXML(const LASHeaderH hHeader);

/****************************************************************************/
/* Writer Operations                                                        */
/****************************************************************************/

/** Creates a new LASWriterH for write operations on LAS files.  The file may 
 *  be opened in either LAS_MODE_APPEND or LAS_MODE_WRITE, but the file cannot 
 *  be open by another other operations (another LASReaderH or LASWriterH).  
 *  @param filename The filename to open to write
 *  @param hHeader an opaque pointer to a LASHeaderH that will be written to 
 *  the file as part of the opening for write operation.
 *  @param mode a mode value to denote whether to open for write or append 
 *  operations.  Valid values are LAS_MODE_APPEND and LAS_MODE_WRITE.
*/
LAS_DLL LASWriterH LASWriter_Create(const char* filename, const LASHeaderH hHeader, int mode);

/** Writes a point to the file.  The location of where the point is writen is 
 *  determined by the mode the file is opened in, and what the last operation was.  
 *  For example, if the file was opened for append, the next point would be written 
 *  at the end of the file.  Likewise, if the file is opened in write mode, even 
 *  if the file already existed, the next WritePoint operation will happen at the 
 *  end of the header and all of the existing points in the file will be overwritten.
 *  @param hWriter opaque pointer to the LASWriterH instance
 *  @param hPoint the opaque LASPointH pointer to write
 *  @return LE_None if no error occurred during the write operation.
*/
LAS_DLL LASError LASWriter_WritePoint(const LASWriterH hWriter, const LASPointH hPoint);

/** Overwrites the header for the file represented by the LASWriterH.  It does 
 *  not matter if the file is opened for append or for write.  This function is 
 *  equivalent to calling LASWriter_SetHeader and LASWriter_WriteOwnedHeader
 *  simultaneously.
 *  @param hWriter opaque pointer to the LASWriterH instance
 *  @param hHeader LASHeaderH instance to write into the file
 *  @return LE_None if no error occurred during the operation.
*/

LAS_DLL LASError LASWriter_WriteHeader(const LASWriterH hWriter, const LASHeaderH hHeader);

/** Overwrites the header for the file represented by the LASWriterH that was 
 *  set using LASWriter_SetHeader or flushes the existing header that is on the
 *  the writer to the file and resets the file for writing.
 *  @param hWriter opaque pointer to the LASWriterH instance
 *  @return LE_None if no error occurred during the operation.
*/

LAS_DLL LASError LASWriter_WriteOwnedHeader(const LASWriterH hWriter);

/** Destroys the LASWriterH instance, effectively closing the file and performing 
 *  housekeeping operations.
 *  @param hWriter LASWriterH instance to close
*/
LAS_DLL void LASWriter_Destroy(LASWriterH hWriter);

/** Returns a LASHeaderH representing the header for the file
 *  @param hWriter the LASWriterH instance
 *  @return a LASHeaderH representing the header for the file.  NULL is returned 
 *  in the event of an error.  Use the LASError_GetLastError* methods to check
 *  in the event of a NULL return.
*/
LAS_DLL LASHeaderH LASWriter_GetHeader(const LASWriterH hWriter);
LAS_DLL void LASWriter_SetHeader(  LASWriterH hWriter, const LASHeaderH hHeader) ;

LAS_DLL LASError LASWriter_SetSRS(LASWriterH hWriter, const LASSRSH hSRS);
LAS_DLL LASError LASWriter_SetInputSRS(LASWriterH hWriter, const LASSRSH hSRS);
LAS_DLL LASError LASWriter_SetOutputSRS(LASWriterH hWriter, const LASSRSH hSRS);

/****************************************************************************/
/* GUID Operations                                                          */
/****************************************************************************/

/** Returns the GUID value for the header as an opaque LASGuidH pointer.
 *  @param hHeader the opaque pointer to the LASHeaderH
 *  @return the GUID value for the header as an opaque LASGuidH pointer.
*/
LAS_DLL LASGuidH LASHeader_GetGUID(const LASHeaderH hHeader);

/** Returns a new random GUID.
 *  @return a new random GUID
*/
LAS_DLL LASGuidH LASGuid_Create();

/** Creates a new GUID opaque pointer using the given string.  
 *  @param string A GUID string in the form "00000000-0000-0000-0000-000000000000"
 *  An example GUID might be something like '8388F1B8-AA1B-4108-BCA3-6BC68E7B062E'
 *  @return the GUID value as an opaque LASGuidH pointer.
*/
LAS_DLL LASGuidH LASGuid_CreateFromString(const char* string);

/** Destroys a GUID opaque pointer and removes it from the heap
 *  @param hId the GUID value to destroy as an opaque LASGuidH pointer.
*/
LAS_DLL void LASGuid_Destroy(LASGuidH hId);

/** Determines if two GUIDs are equal.
 *  @param hId1 the first GUID
 *  @param hId2 the second GUID
 *  @return 0 if false, 1 if true.  Use the LASError_GetLastError* methods to 
 *  determine if an error occured during the operation of this function.
*/
LAS_DLL int LASGuid_Equals(LASGuidH hId1, LASGuidH hId2);

/** Returns a string representation of the GUID opqaue pointer.  The caller 
 *  owns the string.
 *  @param hId the LASGuidH pointer
 *  @return a string representation of the GUID opaque pointer.
*/
LAS_DLL char* LASGuid_AsString(LASGuidH hId);

/****************************************************************************/
/* VLR Operations                                                           */
/****************************************************************************/

/** Creates a new VLR record
 *  @return a new VLR record
*/
LAS_DLL LASVLRH LASVLR_Create(void);

/** Destroys a VLR record and removes it from the heap
*/
LAS_DLL void LASVLR_Destroy(LASVLRH hVLR);

/** Returns the User Id for the VLR 
 *  @param hVLR the LASVLRH instance
 *  @return the User Id for the VLR
*/
LAS_DLL char* LASVLR_GetUserId(const LASVLRH hVLR);

/** Sets the User Id for the VLR
 *  @param hVLR the LASVLRH instance
 *  @param value the value to set for the User Id.  It will be clipped to fit 
 *  within 16 characters
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetUserId(LASVLRH hVLR, const char* value);

/** Gets the description for the VLR
 *  @param hVLR the LASVLRH instance
 *  @return the description for the VLR
*/ 
LAS_DLL char* LASVLR_GetDescription(const LASVLRH hVLR);

/** Sets the description for the VLR
 *  @param hVLR the LASVLRH instance
 *  @param value the value to set for the description.  It will be clipped to fit 
 *  within 32 characters
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetDescription(LASVLRH hVLR, const char* value);

/** Returns the record length of the data stored in the VLR
 *  @param hVLR the LASVLRH instance
 *  @return the record length of the data stored in the VLR
*/
LAS_DLL unsigned short LASVLR_GetRecordLength(const LASVLRH hVLR);

/** Sets the record length of the data stored in the VLR
 *  @param hVLR the LASVLRH instance
 *  @param value the length to set for the VLR data length
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetRecordLength(LASVLRH hVLR, unsigned short value);

/** Gets the record id for the VLR
 *  @param hVLR the LASVLRH instance
 *  @return the record id for the VLR
*/
LAS_DLL unsigned short LASVLR_GetRecordId(const LASVLRH hVLR);

/** Sets the record id for the VLR
 *  @param hVLR the LASVLRH instance
 *  @param value the record id to set
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetRecordId(LASVLRH hVLR, unsigned short value);

/** Gets the reserved value of the VLR.  This should be 0 and should aways be 0.
 *  @param hVLR the LASVLRH instance
 *  @return the reserved value of the VLR.
*/
LAS_DLL unsigned short LASVLR_GetReserved(const LASVLRH hVLR);

/** Sets the reserved value of the VLR.  This should be 0 and you should not 
 *  have to ever monkey with this value according to the spec.
 *  @param hVLR the LASVLRH instance
 *  @param value the value to set for the reserved value
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetReserved(LASVLRH hVLR, unsigned short value);

/** Gets the data stream for the VLR as an array of bytes.  The length of this 
 *  array should be the same as LASVLR_GetRecordLength.  You must allocate it on 
 *  the heap and you are responsible for its destruction.
 *  @param hVLR the LASVLRH instance
 *  @param data a pointer to your array where you want the data copied
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_GetData(const LASVLRH hVLR, unsigned char* data);

/** Sets the data stream for the VLR as an array of bytes.  The length of this 
 *  array should be the same as LASVLR_GetRecordLength.  The data are copied into 
 *  the VLR structure.
 *  @param hVLR the LASVLRH instance
 *  @param data a pointer to your array.  It must be LASVLR_GetRecordLength in size
 *  @param length length of the data to set on the VLR
 *  @return LASErrorEnum
*/
LAS_DLL LASError LASVLR_SetData(const LASVLRH hVLR, unsigned char* data, unsigned short length);


/****************************************************************************/
/* Color Operations                                                           */
/****************************************************************************/

/** Creates a new Color
 *  @return a new Color
*/
LAS_DLL LASColorH LASColor_Create(void);

/** Destroys a Color and removes it from the heap
*/
LAS_DLL void LASColor_Destroy(LASColorH hColor);

/** Returns the red value for the color.
 *  @return the red value for the color.
*/
LAS_DLL unsigned short LASColor_GetRed(const LASColorH hColor);

/** Sets the red value for the color
 *  @param hColor the opaque pointer to the LASColorH instance
 *  @param value the value to set the red value to
 *  @return an error number if an error occured.
*/
LAS_DLL LASError LASColor_SetRed(LASColorH hColor, unsigned short value);

/** Returns the green value for the color.
 *  @return the green value for the color.
*/
LAS_DLL unsigned short LASColor_GetGreen(const LASColorH hColor);

/** Sets the green value for the color
 *  @param hColor the opaque pointer to the LASColorH instance
 *  @param value the value to set the green value to
 *  @return an error number if an error occured.
*/
LAS_DLL LASError LASColor_SetGreen(LASColorH hColor, unsigned short value);

/** Returns the blue value for the color.
 *  @return the blue value for the color.
*/
LAS_DLL unsigned short LASColor_GetBlue(const LASColorH hColor);

/** Sets the blue value for the color
 *  @param hColor the opaque pointer to the LASColorH instance
 *  @param value the value to set the blue value to
 *  @return an error number if an error occured.
*/
LAS_DLL LASError LASColor_SetBlue(LASColorH hColor, unsigned short value);


/** Returns the color for the LASPointH
 *  @return the color for the LASPointH.
*/
LAS_DLL LASColorH LASPoint_GetColor(const LASPointH hPoint);

/** Sets the color for the point
 *  @param hPoint the opaque pointer to the LASPointH instance
 *  @param hColor the opaque pointer to the LASColorH instance
 *  @return an error number if an error occured.
*/
LAS_DLL LASError LASPoint_SetColor(LASPointH hPoint, const LASColorH hColor);


/****************************************************************************/
/* SRS Operations                                                           */
/****************************************************************************/

/** Creates a new SRS
 *  @return a new SRS
*/
LAS_DLL LASSRSH LASSRS_Create(void);

LAS_DLL const /*GTIF*/ void* LASSRS_GetGTIF(LASSRSH hSRS);
LAS_DLL LASError LASSRS_SetGTIF(LASSRSH hSRS, const /* GTIF */ void* pgtiff, const /* ST_TIFF */ void* ptiff);
LAS_DLL char* LASSRS_GetWKT(LASSRSH hSRS );
LAS_DLL char* LASSRS_GetWKT_CompoundOK( LASSRSH hSRS );
LAS_DLL LASError LASSRS_SetWKT(LASSRSH hSRS, const char* value);
LAS_DLL LASError LASSRS_SetFromUserInput(LASSRSH hSRS, const char* value);
LAS_DLL char* LASSRS_GetProj4(LASSRSH hSRS);
LAS_DLL LASError LASSRS_SetProj4(LASSRSH hSRS, const char* value);
LAS_DLL LASError LASSRS_SetVerticalCS(LASSRSH hSRS, int verticalCSType,
                                      const char *citation, int verticalDatum,
                                      int verticalUnits );
LAS_DLL LASSRSH LASHeader_GetSRS(const LASHeaderH hHeader);
LAS_DLL LASError LASHeader_SetSRS(LASHeaderH hHeader, const LASSRSH hSRS);
LAS_DLL void LASSRS_Destroy(LASSRSH hSRS);
LAS_DLL LASVLRH LASSRS_GetVLR(const LASSRSH hSRS, unsigned int i);
LAS_DLL unsigned int LASSRS_GetVLRCount(const LASSRSH hSRS);

/** Method to ensure that you are freeing char*'s from the 
 *  correct heap.
 *  @param string the string to free
*/
LAS_DLL void LASString_Free(char* string);

LAS_DLL unsigned int LASSchema_GetByteSize( LASSchemaH hFormat);
LAS_DLL unsigned int LASSchema_GetBaseByteSize( LASSchemaH hFormat);



LAS_DLL LASSchemaH LASHeader_GetSchema( LASHeaderH hHeader );
LAS_DLL LASError LASHeader_SetSchema( LASHeaderH hHeader, LASSchemaH hFormat);

LAS_DLL void LASSchema_Destroy(LASSchemaH hFormat);

/** Returns the compression status of the header
 *  @param hHeader LASHeaderH instance
 *  @return 1 if the file is compressed, 0 otherwise
 */
LAS_DLL int LASHeader_Compressed(const LASHeaderH hHeader);

/** Sets the compression status of the header
 *  @param hHeader LASHeaderH instance
 *  @param b 1if the file is compressed, 0 otherwise
 *  @return LASerror enum
 */
LAS_DLL LASError LASHeader_SetCompressed(LASHeaderH hHeader, int b);

LAS_C_END
#endif

