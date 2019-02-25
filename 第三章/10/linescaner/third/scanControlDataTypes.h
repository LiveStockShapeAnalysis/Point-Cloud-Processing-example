//_______________________________________________________________________
// Datentypen, Defines fuer die LLTs
//
// $Source: /Entwicklung/Projekte/LLT2800/PC/Firewire/Llt/scanControlDataTypes.h,v $
// $Id: scanControlDataTypes.h,v 1.6 2013/08/29 10:44:23 17000038 Exp $
//
// Sven Ackermann
// (c) 2008 MICRO-EPSILON Optronic GmbH
//_________________________________________________________________________
//

#ifndef _SCANCONTROLLDATENTYPEN_H_
#define _SCANCONTROLLDATENTYPEN_H_


//Message-Values
#define ERROR_OK                                            0
#define ERROR_SERIAL_COMM                                   1
#define ERROR_SERIAL_LLT                                    7
#define ERROR_CONNECTIONLOST                                10
#define ERROR_STOPSAVING                                    100

//specify the type for the RS422-multiplexer
#define RS422_INTERFACE_FUNCTION_AUTO                       0
#define RS422_INTERFACE_FUNCTION_SERIALPORT_115200          1
#define RS422_INTERFACE_FUNCTION_TRIGGER                    2
#define RS422_INTERFACE_FUNCTION_CMM_TRIGGER                3
#define RS422_INTERFACE_FUNCTION_ENCODER                    4
#define RS422_INTERFACE_FUNCTION_DIGITAL_OUTPUT             6
#define RS422_INTERFACE_FUNCTION_TRIGGER_LASER_PULSE        7
#define RS422_INTERFACE_FUNCTION_SERIALPORT_57600           8
#define RS422_INTERFACE_FUNCTION_SERIALPORT_38400           9
#define RS422_INTERFACE_FUNCTION_SERIALPORT_19200           10
#define RS422_INTERFACE_FUNCTION_SERIALPORT_9600            11

//Return-Values for the is-functions
#define IS_FUNC_NO                                          0
#define IS_FUNC_YES                                         1

//General return-values for all functions
#define GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED          4
#define GENERAL_FUNCTION_PACKET_SIZE_CHANGED                3
#define GENERAL_FUNCTION_CONTAINER_MODE_HEIGHT_CHANGED      2
#define GENERAL_FUNCTION_OK                                 1
#define GENERAL_FUNCTION_NOT_AVAILABLE                      0

#define ERROR_GENERAL_WHILE_LOAD_PROFILE                    -1000
#define ERROR_GENERAL_NOT_CONNECTED                         -1001
#define ERROR_GENERAL_DEVICE_BUSY                           -1002
#define ERROR_GENERAL_WHILE_LOAD_PROFILE_OR_GET_PROFILES    -1003
#define ERROR_GENERAL_WHILE_GET_PROFILES                    -1004
#define ERROR_GENERAL_GET_SET_ADDRESS                       -1005
#define ERROR_GENERAL_POINTER_MISSING                       -1006
#define ERROR_GENERAL_WHILE_SAVE_PROFILES                   -1007
#define ERROR_GENERAL_SECOND_CONNECTION_TO_LLT              -1008

//Return-Values for GetDeviceName
#define ERROR_GETDEVICENAME_SIZE_TOO_LOW                    -1
#define ERROR_GETDEVICENAME_NO_BUFFER                       -2

//Return-Values for Load/SaveProfiles
#define ERROR_LOADSAVE_WRITING_LAST_BUFFER                  -50
#define ERROR_LOADSAVE_WHILE_SAVE_PROFILE                   -51
#define ERROR_LOADSAVE_NO_PROFILELENGTH_POINTER             -52
#define ERROR_LOADSAVE_NO_LOAD_PROFILE                      -53
#define ERROR_LOADSAVE_STOP_ALREADY_LOAD                    -54
#define ERROR_LOADSAVE_CANT_OPEN_FILE                       -55
#define ERROR_LOADSAVE_INVALID_FILE_HEADER                  -56
#define ERROR_LOADSAVE_FILE_POSITION_TOO_HIGH               -57
#define ERROR_LOADSAVE_AVI_NOT_SUPPORTED                    -58
#define ERROR_LOADSAVE_NO_REARRANGEMENT_POINTER             -59
#define ERROR_LOADSAVE_WRONG_PROFILE_CONFIG                 -60
#define ERROR_LOADSAVE_NOT_TRANSFERING                      -61

//Return-Values for profile transfer functions
#define ERROR_PROFTRANS_SHOTS_NOT_ACTIVE                    -100
#define ERROR_PROFTRANS_SHOTS_COUNT_TOO_HIGH                -101
#define ERROR_PROFTRANS_WRONG_PROFILE_CONFIG                -102
#define ERROR_PROFTRANS_FILE_EOF                            -103
#define ERROR_PROFTRANS_NO_NEW_PROFILE                      -104
#define ERROR_PROFTRANS_BUFFER_SIZE_TOO_LOW                 -105
#define ERROR_PROFTRANS_NO_PROFILE_TRANSFER                 -106
#define ERROR_PROFTRANS_PACKET_SIZE_TOO_HIGH                -107
#define ERROR_PROFTRANS_CREATE_BUFFERS                      -108
#define ERROR_PROFTRANS_WRONG_PACKET_SIZE_FOR_CONTAINER     -109
#define ERROR_PROFTRANS_REFLECTION_NUMBER_TOO_HIGH          -110
#define ERROR_PROFTRANS_MULTIPLE_SHOTS_ACTIV                -111
#define ERROR_PROFTRANS_BUFFER_HANDOUT                      -112
#define ERROR_PROFTRANS_WRONG_BUFFER_POINTER                -113
#define ERROR_PROFTRANS_WRONG_TRANSFER_CONFIG               -114

//Return-Values for Set/GetFunctions
#define ERROR_SETGETFUNCTIONS_WRONG_BUFFER_COUNT            -150
#define ERROR_SETGETFUNCTIONS_PACKET_SIZE                   -151
#define ERROR_SETGETFUNCTIONS_WRONG_PROFILE_CONFIG          -152
#define ERROR_SETGETFUNCTIONS_NOT_SUPPORTED_RESOLUTION      -153
#define ERROR_SETGETFUNCTIONS_REFLECTION_NUMBER_TOO_HIGH    -154
#define ERROR_SETGETFUNCTIONS_WRONG_FEATURE_ADRESS          -155
#define ERROR_SETGETFUNCTIONS_SIZE_TOO_LOW                  -156
#define ERROR_SETGETFUNCTIONS_WRONG_PROFILE_SIZE            -157
#define ERROR_SETGETFUNCTIONS_MOD_4                         -158
#define ERROR_SETGETFUNCTIONS_REARRANGEMENT_PROFILE         -159
#define ERROR_SETGETFUNCTIONS_USER_MODE_TOO_HIGH            -160
#define ERROR_SETGETFUNCTIONS_USER_MODE_FACTORY_DEFAULT     -161
#define ERROR_SETGETFUNCTIONS_HEARTBEAT_TOO_HIGH            -162


//Return-Values PostProcessingParameter
#define ERROR_POSTPROCESSING_NO_PROF_BUFFER                 -200
#define ERROR_POSTPROCESSING_MOD_4                          -201
#define ERROR_POSTPROCESSING_NO_RESULT                      -202
#define ERROR_POSTPROCESSING_LOW_BUFFERSIZE                 -203
#define ERROR_POSTPROCESSING_WRONG_RESULT_SIZE              -204

//Return-Values for GetDeviceInterfaces
#define ERROR_GETDEVINTERFACES_WIN_NOT_SUPPORTED            -250
#define ERROR_GETDEVINTERFACES_REQUEST_COUNT                -251
#define ERROR_GETDEVINTERFACES_CONNECTED                    -252
#define ERROR_GETDEVINTERFACES_INTERNAL                     -253

//Return-Values for Connect
#define ERROR_CONNECT_LLT_COUNT                             -300
#define ERROR_CONNECT_SELECTED_LLT                          -301
#define ERROR_CONNECT_ALREADY_CONNECTED                     -302
#define ERROR_CONNECT_LLT_NUMBER_ALREADY_USED               -303
#define ERROR_CONNECT_SERIAL_CONNECTION                     -304
#define ERROR_CONNECT_INVALID_IP                            -305

//Return-Values for SetPartialProfile
#define ERROR_PARTPROFILE_NO_PART_PROF                      -350
#define ERROR_PARTPROFILE_TOO_MUCH_BYTES                    -351
#define ERROR_PARTPROFILE_TOO_MUCH_POINTS                   -352
#define ERROR_PARTPROFILE_NO_POINT_COUNT                    -353
#define ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_POINT            -354
#define ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_DATA             -355

//Return-Values for Start/StopTransmissionAndCmmTrigger
#define ERROR_CMMTRIGGER_NO_DIVISOR                         -400
#define ERROR_CMMTRIGGER_TIMEOUT_AFTER_TRANSFERPROFILES     -401
#define ERROR_CMMTRIGGER_TIMEOUT_AFTER_SETCMMTRIGGER        -402

//Return-Values for TranslateErrorValue
#define ERROR_TRANSERRORVALUE_WRONG_ERROR_VALUE             -450
#define ERROR_TRANSERRORVALUE_BUFFER_SIZE_TOO_LOW           -451

//Read/write config functions
#define ERROR_READWRITECONFIG_CANT_CREATE_FILE              -500
#define ERROR_READWRITECONFIG_CANT_OPEN_FILE                -501
#define ERROR_READWRITECONFIG_QUEUE_TO_SMALL                -502

//Function defines for the Get/SetFeature function
#define FEATURE_FUNCTION_SERIAL                             0xf0000410
#define FEATURE_FUNCTION_LASERPOWER                         0xf0f00824
#define INQUIRY_FUNCTION_LASERPOWER                         0xf0f00524
#define FEATURE_FUNCTION_MEASURINGFIELD                     0xf0f00880
#define INQUIRY_FUNCTION_MEASURINGFIELD                     0xf0f00580
#define FEATURE_FUNCTION_TRIGGER                            0xf0f00830
#define INQUIRY_FUNCTION_TRIGGER                            0xf0f00530
#define FEATURE_FUNCTION_SHUTTERTIME                        0xf0f0081c
#define INQUIRY_FUNCTION_SHUTTERTIME                        0xf0f0051c
#define FEATURE_FUNCTION_IDLETIME                           0xf0f00800
#define INQUIRY_FUNCTION_IDLETIME                           0xf0f00500
#define FEATURE_FUNCTION_PROCESSING_PROFILEDATA             0xf0f00804
#define INQUIRY_FUNCTION_PROCESSING_PROFILEDATA             0xf0f00504
#define FEATURE_FUNCTION_THRESHOLD                          0xf0f00810
#define INQUIRY_FUNCTION_THRESHOLD                          0xf0f00510
#define FEATURE_FUNCTION_MAINTENANCEFUNCTIONS               0xf0f0088c
#define INQUIRY_FUNCTION_MAINTENANCEFUNCTIONS               0xf0f0058c
#define FEATURE_FUNCTION_ANALOGFREQUENCY                    0xf0f00828
#define INQUIRY_FUNCTION_ANALOGFREQUENCY                    0xf0f00528
#define FEATURE_FUNCTION_ANALOGOUTPUTMODES                  0xf0f00820
#define INQUIRY_FUNCTION_ANALOGOUTPUTMODES                  0xf0f00520
#define FEATURE_FUNCTION_CMMTRIGGER                         0xf0f00888
#define INQUIRY_FUNCTION_CMMTRIGGER                         0xf0f00588
#define FEATURE_FUNCTION_REARRANGEMENT_PROFILE              0xf0f0080c
#define INQUIRY_FUNCTION_REARRANGEMENT_PROFILE              0xf0f0050c
#define FEATURE_FUNCTION_PROFILE_FILTER                     0xf0f00818
#define INQUIRY_FUNCTION_PROFILE_FILTER                     0xf0f00518
#define FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION           0xf0f008c0
#define INQUIRY_FUNCTION_RS422_INTERFACE_FUNCTION           0xf0f005c0

#define FEATURE_FUNCTION_SATURATION                         0xf0f00814
#define INQUIRY_FUNCTION_SATURATION                         0xf0f00514
#define FEATURE_FUNCTION_TEMPERATURE                        0xf0f0082c
#define INQUIRY_FUNCTION_TEMPERATURE                        0xf0f0052c
#define FEATURE_FUNCTION_CAPTURE_QUALITY                    0xf0f008c4
#define INQUIRY_FUNCTION_CAPTURE_QUALITY                    0xf0f005c4
#define FEATURE_FUNCTION_SHARPNESS                          0xf0f00808
#define INQUIRY_FUNCTION_SHARPNESS                          0xf0f00508

#define FEATURE_FUNCTION_PACKET_DELAY                       0x00000d08
#define FEATURE_FUNCTION_CONNECTION_SPEED                   0x00000670

//Defines for the return values of the ConvertProfile2Values and ConvertPartProfile2Values functions
#define CONVERT_WIDTH                                       0x00000100
#define CONVERT_MAXIMUM                                     0x00000200
#define CONVERT_THRESHOLD                                   0x00000400
#define CONVERT_X                                           0x00000800
#define CONVERT_Z                                           0x00001000
#define CONVERT_M0                                          0x00002000
#define CONVERT_M1                                          0x00004000

typedef void (_stdcall *TNewProfile)(const unsigned char *pData, unsigned int nSize);
typedef void (_stdcall *TNewProfile_s)(const unsigned char *pData, unsigned int nSize, void *pUserData);
typedef void (_cdecl *TNewProfile_c)(const unsigned char *pData, unsigned int nSize, void *pUserData);

//specify the type of the scanner
//if you programming language don't support enums, you can use a signed int
typedef enum TScannerType
{
  StandardType = -1,                    //can't decode scanCONTROL name use standard measurmentrange
  LLT25  = 0,                           //scanCONTROL28xx with 25mm measurmentrange
  LLT100 = 1,                           //scanCONTROL28xx with 100mm measurmentrange
  
  scanCONTROL28xx_25  = 0,              //scanCONTROL28xx with 25mm measurmentrange
  scanCONTROL28xx_100 = 1,              //scanCONTROL28xx with 100mm measurmentrange
  scanCONTROL28xx_10  = 2,              //scanCONTROL28xx with 10mm measurmentrange
  scanCONTROL28xx_xxx = 999,            //scanCONTROL28xx with no measurmentrange -> use standard measurmentrange
 
  scanCONTROL27xx_25  = 1000,           //scanCONTROL27xx with 25mm measurmentrange
  scanCONTROL27xx_100 = 1001,           //scanCONTROL27xx with 100mm measurmentrange
  scanCONTROL27xx_50  = 1002,           //scanCONTROL27xx with 50mm measurmentrange
  scanCONTROL2751_25  = 1020,           //scanCONTROL27xx with 25mm measurmentrange
  scanCONTROL2751_100 = 1021,           //scanCONTROL27xx with 100mm measurmentrange
  scanCONTROL2702_50  = 1032,           //scanCONTROL2702 with 50mm measurement range
  scanCONTROL27xx_xxx = 1999,           //scanCONTROL27xx with no measurmentrange -> use standard measurmentrange

  scanCONTROL26xx_25  = 2000,           //scanCONTROL26xx with 25mm measurmentrange
  scanCONTROL26xx_100 = 2001,           //scanCONTROL26xx with 100mm measurmentrange
  scanCONTROL26xx_50  = 2002,           //scanCONTROL26xx with 50mm measurmentrange
  scanCONTROL2651_25  = 2020,           //scanCONTROL26xx with 25mm measurmentrange
  scanCONTROL2651_100 = 2021,           //scanCONTROL26xx with 100mm measurmentrange
  scanCONTROL2602_50  = 2032,           //scanCONTROL2602 with 50mm measurement range
  scanCONTROL26xx_xxx = 2999,           //scanCONTROL26xx with no measurmentrange -> use standard measurmentrange

  scanCONTROL29xx_25  = 3000,           //scanCONTROL29xx with 25mm measurmentrange
  scanCONTROL29xx_100 = 3001,           //scanCONTROL29xx with 100mm measurmentrange
  scanCONTROL29xx_50  = 3002,           //scanCONTROL29xx with 50mm measurmentrange
  scanCONTROL2951_25  = 3020,           //scanCONTROL29xx with 25mm measurmentrange
  scanCONTROL2951_100 = 3021,           //scanCONTROL29xx with 100mm measurmentrange
  scanCONTROL2902_50  = 3032,           //scanCONTROL2902 with 50mm measurement range
  scanCONTROL2953_30  = 3033,           //scanCONTROL2953 with 30mm measurement range
  scanCONTROL29xx_xxx = 3999,           //scanCONTROL29xx with no measurmentrange -> use standard measurmentrange
  scanCONTROL2xxx     = 4000,           //scanCONTROL2xxx with 100mm measurmentrange
}TScannerType;

//specify the profile configuration
//if you programming language don't support enums, you can use a signed int
typedef enum TProfileConfig
{
  NONE            = 0,
  PROFILE         = 1,
  CONTAINER       = 1,
  VIDEO_IMAGE     = 1,
  PURE_PROFILE    = 2,
  QUARTER_PROFILE = 3,
  CSV_PROFILE     = 4,
  PARTIAL_PROFILE = 5,
}TProfileConfig;

//specify the callback type
//if you programming language don't support enums, you can use a signed int
typedef enum TCallbackType
{
  STD_CALL = 0,
  C_DECL   = 1,
}TCallbackType;

//specify the file type for saving
//if you programming language don't support enums, you can use a signed int
typedef enum TFileType
{
  AVI = 0,
  LLT = 1,
  CSV = 2,
  BMP = 3,
  CSV_NEG = 4,
}TFileType;

//Structure for the partial profile parameters
typedef struct TPartialProfile
{
  unsigned int nStartPoint;
  unsigned int nStartPointData;
  unsigned int nPointCount;
  unsigned int nPointDataWidth;
}TPartialProfile;

typedef enum TTransferProfileType
{
  NORMAL_TRANSFER = 0,
  SHOT_TRANSFER   = 1,
  NORMAL_CONTAINER_MODE = 2,
  SHOT_CONTAINER_MODE = 3,
  NONE_TRANSFER = 4,
}TTransferProfileType;

typedef enum TTransferVideoType
{
  VIDEO_MODE_0 = 0,
  VIDEO_MODE_1 = 1,
  NONE_VIDEOMODE = 2,
}TTransferVideoType;

typedef enum TInterfaceType
{
  INTF_TYPE_UNKNOWN  = 0,
  INTF_TYPE_SERIAL   = 1,
  INTF_TYPE_FIREWIRE = 2,
  INTF_TYPE_ETHERNET = 3
} TInterfaceType;

#endif	// _SCANCONTROLLDATENTYPEN_H_
