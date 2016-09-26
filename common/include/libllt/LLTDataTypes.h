#ifndef LLTDATATYPES_H
#define LLTDATATYPES_H

// Returnvals
#define GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED	4
#define GENERAL_FUNCTION_CONTAINER_MODE_HEIGHT_CHANGED	2	   // Function successful, but height for container mode changed
#define GENERAL_FUNCTION_OK				1	   // Function executed successfully
#define GENERAL_FUNCTION_NOT_AVAILABLE			0	   // Function not available

// Errorcodes
#define ERROR_GENERAL_WHILE_LOAD_PROFILE		-1000	   // Function couldn't be loaded for current mode
#define ERROR_GENERAL_NOT_CONNECTED			-1001	   // No connection to scanCONTROL
#define ERROR_GENERAL_DEVICE_BUSY			-1002	   // Connection to scanCONTROL lost or interrupted
#define ERROR_GENERAL_WHILE_LOAD_PROFILE_OR_GET_PROFILES -1003	   // Function couldnt be executed due to active profile operation
#define ERROR_GENERAL_WHILE_GET_PROFILES		-1004	   // Function couldnt be executed due to active profile transmission
#define ERROR_GENERAL_GET_SET_ADDRESS			-1005	   // No write or read operation at address available
#define ERROR_GENERAL_POINTER_MISSING			-1006	   // A necessary pointer is NULL
#define ERROR_GENERAL_WHILE_SAVE_PROFILES		-1007	   // Function couldnt be executed due to active profile saving

#define ERROR_CONNECT_LLT_COUNT				-400	   // No scanner connected
#define ERROR_CONNECT_SELECTED_LLT			-401	   // Selected interface not available
#define ERROR_CONNECT_LLT_NUMBER_ALREADY_USED		-404	   // Selected LLT already connected

#define ERROR_GETDEVINTERFACE_REQUEST_COUNT		-251
#define ERROR_GETDEVINTERFACE_INTERNAL			-253	   // Failure during scanCONTROL request
#define ERROR_GETDEVICENAME_SIZE_TOO_LOW		-1	   // Buffer size to small
#define ERROR_GETDEVICENAME_NO_BUFFER			-2	   // No buffer for name and/or vendor

#define ERROR_SETGETFUNCTIONS_WRONG_BUFFER_COUNT	-150	   // Buffercont has to be between 2 and 200
#define ERROR_SETGETFUNCTIONS_PACKET_SIZE		-151	   // Wrong packet size
#define ERROR_SETGETFUNCTIONS_WRONG_PROFILE_CONFIG	-152	   // Wrong profile config
#define ERROR_SETGETFUNCTIONS_NOT_SUPPORTED_RESOLUTION	-153	   // Wrong resolution
#define ERROR_SETGETFUNCTIONS_REFLECTION_NUMBER_TOO_HIGH -154	   // Index of reflection is higher than 3
#define ERROR_SETGETFUNCTIONS_WRONG_FEATURE_ADRESS	-155	   // Wrong Adress
#define ERROR_SETGETFUNCTIONS_SIZE_TOO_LOW		-156	   // Size of res field to low
#define ERROR_SETGETFUNCTIONS_WRONG_PROFILE_SIZE	-157	   // Wrong container size
#define ERROR_SETGETFUNCTIONS_MOD_4			-158	   // Container breite ist nicht durch 4 teilbar
#define ERROR_SETGETFUNCTIONS_USER_MODE_TOO_HIGH	-160	   // Wrong user mode number
#define ERROR_SETGETFUNCITONS_USER_MODE_FACTORY_DEFAULT -161	   // Usermode 0 kann nicht überschrieben werden (Standard)
#define ERROR_SETGETFUNCITONS_HEARTBEAT_TOO_HIGH	-162	   // Heartbeatparameter zu groß

#define ERROR_PARTPROFILE_NO_PART_PROF			-350	   // Wrong profile config for pp
#define ERROR_PARTPROFILE_TOO_MUCH_BYTES		-351
#define ERROR_PARTPROFILE_TOO_MUCH_POINTS		-352
#define ERROR_PARTPROFILE_NO_POINT_COUNT		-353
#define ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_POINT	-354
#define ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_DATA		-355	

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

#define ERROR_DEVPROP_NOT_AVAILABLE  			-999 	   // No filename for devproperties given
#define ERROR_DEVPROP_NOT_FOUND				-1300
#define ERROR_DEVPROP_DECODE				-1301
#define ERROR_DEVPROP_DEPRECATED			-1302
#define ERROR_DEVPROP_READ_FAILURE			-1303

#define ERROR_TRANSMISSION_CANCEL_NO_CAM 		-888
#define ERROR_TRANSMISSION_CANCEL_NO_TRANSMISSION_ACTIVE -889
		
// Status and Control Register
#define FEATURE_FUNCTION_SERIAL				0xf0000410 // Serial
#define FEATURE_FUNCTION_LASERPOWER			0xf0f00824 // Laser Power
#define FEATURE_FUNCTION_MEASURINGFIELD			0xf0f00880 // Measuring Field
#define FEATURE_FUNCTION_TRIGGER			0xf0f00830 // Trigger
#define FEATURE_FUNCTION_SHUTTERTIME			0xf0f0081c // Shutter time
#define FEATURE_FUNCTION_IDLETIME			0xf0f00800 // Idle time
#define FEATURE_FUNCTION_PROCESSINGPROFILEDATA		0xf0f00804 // Processing Profile Data
#define FEATURE_FUNCTION_THRESHOLD			0xf0f00810 // Threshold
#define FEATURE_FUNCTION_MAINTENANCEFUNCTIONS		0xf0f0088c // Maintenance functions
#define FEATURE_FUNCTION_ANALOGFREQUENCY		0xf0f00828 // Analog frequency
#define FEATURE_FUNCTION_ANALOGOUTPUTMODES		0xf0f00820 // Analog output modes
#define FEATURE_FUNCTION_CMMTRIGGER			0xf0f00888 // CMM Trigger
#define FEATURE_FUNCTION_REARRANGEMENT_PROFILE		0xf0f0080c // Rearrangement profile
#define FEATURE_FUNCTION_PROFILE_FILTER			0xf0f00818 // Profile filter
#define FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION 	0xf0f008c0 // RS422 Interface Function
#define FEATURE_FUNCTION_PACKET_DELAY			0x00000d08 // Packet delay
#define FEATURE_FUNCTION_SHARPNESS			0xf0f00808 // Serial register

// Inquiry Register
#define INQUIRY_FUNCTION_LASERPOWER			0xf0f00524 // Laser Power
#define INQUIRY_FUNCTION_MEASURINGFIELD			0xf0f00580 // Measuring Field
#define INQUIRY_FUNCTION_SHUTTERTIME			0xf0f0051c // Shutter time
#define INQUIRY_FUNCTION_IDLETIME			0xf0f00500 // Idle time
#define INQUIRY_FUNCTION_PROCESSINGPROFILEDATA		0xf0f00504 // Processing Profile Data
#define INQUIRY_FUNCTION_THRESHOLD			0xf0f00510 // Threshold
#define INQUIRY_FUNCTION_MAINTENANCEFUNCTIONS		0xf0f0058c // Maintenance functions
#define INQUIRY_FUNCTION_ANALOGFREQUENCY		0xf0f00828 // Analog frequency
#define INQUIRY_FUNCTION_ANALOGOUTPUTMODES		0xf0f00520 // Analog output modes
#define INQUIRY_FUNCTION_CMMTRIGGER			0xf0f00588 // CMM Trigger
#define INQUIRY_FUNCTION_REARRANGEMENT_PROFILE		0xf0f0050c // Rearrangement profile
#define INQUIRY_FUNCTION_PROFILE_FILTER			0xf0f00518 // Profile filter
#define INQUIRY_FUNCTION_RS422_INTERFACE_FUNCTION 	0xf0f005c0 // RS422 Interface Function
#define INQUIRY_FUNCTION_SHARPNESS			0xf0f00508 // Serial register

#define CONVERT_X  0x00000800
#define CONVERT_Z  0x00001000
#define CONVERT_WIDTH  0x00000100
#define CONVERT_MAXIMUM  0x00000200
#define CONVERT_THRESHOLD  0x00000400
#define CONVERT_M0  0x00002000
#define CONVERT_M1  0x00004000

typedef enum TTransferProfileType
{
	NORMAL_TRANSFER = 0,
	//SHOT_TRANSFER = 1,
	NORMAL_CONTAINER_MODE = 2,
	//SHOT_CONTAINER_MODE = 3,
	NONE_TRANSFER = 4,
}TTransferProfileType;

typedef enum TProfileConfig
{
	NONE = 0,
	PROFILE = 1,
	CONTAINER = 6,
	VIDEO_IMAGE = 2,
	//PURE_PROFILE = 2,
	//QUARTER_PROFILE = 3,
	//CSV_PROFILE = 4,
	PARTIAL_PROFILE = 5,
}TProfileConfig;

//specify the type of the scanner
//if you programming language don't support enums, you can use a signed int
typedef enum TScannerType
{
  StandardType = -1,                    //can't decode scanCONTROL name use standard measurmentrange
  scanCONTROL28xx_xxx = 999,            //scanCONTROL28xx with no measurmentrange -> use standard measurmentrange 
  scanCONTROL27xx_xxx = 1999,           //scanCONTROL27xx with no measurmentrange -> use standard measurmentrange
  scanCONTROL26xx_xxx = 2999,           //scanCONTROL26xx with no measurmentrange -> use standard measurmentrange
  scanCONTROL29xx_xxx = 3999,           //scanCONTROL29xx with no measurmentrange -> use standard measurmentrange
  
}TScannerType;

typedef enum TFileType
{
	AVI = 0,
	//CSV = 2,
	BMP = 3,
	//CSV_NEG = 4,
}TFileType;

#endif // LLTDATATYPES_H
