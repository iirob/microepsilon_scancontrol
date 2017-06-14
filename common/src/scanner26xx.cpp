/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group, 
 * Institute for Anthropomatics and Robotics (IAR) - 
 * Intelligent Process Control and Robotics (IPR), 
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Dennis Hartmann, email: dennis.hartmann@kit.edu
 *
 * Date of creation: 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided
 * that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

#include "scanner26xx.h"

bool Scanner26xx::connect()
{
	if(connected_)
	{
		return true;
	}
	
	std::vector<char *> vcInterfaces(5);
	std::vector<unsigned int> vuiResolutions(10);
	unsigned int uiInterfaceCount = 0;		

	int activeDevice = 0;
	
	int iRetValue = GetDeviceInterfaces(&vcInterfaces[0], vcInterfaces.size());
	if (iRetValue == ERROR_GETDEVINTERFACE_REQUEST_COUNT)
	{
		std::cout << "There are more than " << vcInterfaces.size() << " scanCONTROL connected \n";
		uiInterfaceCount = vcInterfaces.size();
		
	} else if (iRetValue < 1)
	{
		std::cout << "A error occured during searching for connected scanCONTROL \n";
		uiInterfaceCount = 0;
	} else {
		uiInterfaceCount = iRetValue;
		if (uiInterfaceCount == 0)
			std::cout << "There is no scanCONTROL connected \n";
		else if (uiInterfaceCount == 1)
			std::cout << "There is 1 scanCONTROL connected \n";
		else
			std::cout << "There are " << uiInterfaceCount << " scanCONTROL connected \n";
		bool foundSN = false;
		for(int i = 0; i < uiInterfaceCount; ++i)
		{
			std::cout << vcInterfaces[i] << std::endl;
			std::string tempStr = vcInterfaces[i];
			if(serial_number_.size() != 0 && tempStr.compare(tempStr.size() - serial_number_.size(),serial_number_.size(),serial_number_) == 0)
			{
				std::cout << "Found Device with serial number: " << serial_number_ << std::endl;
				foundSN = true;
				activeDevice = i;
				break;
			}
		}
		if(!foundSN && serial_number_.size() != 0)
		{
			std::cout << "Could not find device with S/N: " << serial_number_ << ". Using first device in list." << std::endl;
		}
	}

	
	path_to_device_properties_ += "/device_properties.dat";
        if ((iRetValue = SetPathtoDeviceProperties(path_to_device_properties_.c_str())) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error setting device ID path\nExit program...\n";
		return false;
	}
	
	
	std::cout << "Connecting to " << vcInterfaces[activeDevice] << std::endl;
	if ((llt_.SetDeviceInterface(vcInterfaces[activeDevice])) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting dev id " << iRetValue <<"!\n";
		return false;
	}
	
	/* Connect to sensor */
	if ((iRetValue = llt_.Connect()) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while connecting to camera - Error " << iRetValue <<"!\n";
		return false;
	}
	TScannerType m_tscanCONTROLType;
	if ((iRetValue = llt_.GetLLTType(&m_tscanCONTROLType)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while GetLLTType!\n";
		return false;
	}
	
	if (iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
	{
		std::cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library.\n";
	}
// 	std::cout << llt_.deviceData.device_series << std::endl;
// 	std::cout << llt_.deviceData.scaling << std::endl;
// 	std::cout << llt_.deviceData.offset << std::endl;
// 	std::cout << llt_.deviceData.max_packet_size << std::endl;
// 	std::cout << llt_.deviceData.max_frequency << std::endl;
// 	std::cout << llt_.deviceData.post_proc << std::endl;
// 	std::cout << llt_.deviceData.min_x_display << std::endl;
// 	std::cout << llt_.deviceData.max_x_display << std::endl;
// 	std::cout << llt_.deviceData.min_y_display << std::endl;
// 	std::cout << llt_.deviceData.max_y_display << std::endl;
// 	std::cout << llt_.deviceData.rotate_image << std::endl;
// 	std::cout << llt_.deviceData.min_width << std::endl;
	
	if(m_tscanCONTROLType == scanCONTROL27xx_xxx)
	{
		std::cout << "The scanCONTROL is a scanCONTROL27xx\n";
	}
	else if(m_tscanCONTROLType == scanCONTROL26xx_xxx)
	{
		std::cout << "The scanCONTROL is a scanCONTROL26xx\n";
	}
	else if(m_tscanCONTROLType == scanCONTROL29xx_xxx)
	{
		std::cout << "The scanCONTROL is a scanCONTROL29xx\n";
	}
	else
	{
		std::cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n";
	}
	connected_ = true;
	return true;
}

bool Scanner26xx::initialise()
{
	if(!connected_)
	{
		return false;
	}
	

	//set to default config
	llt_.ReadWriteUserModes(false,0);
	
	if (llt_.SetResolution(scanner_resolution) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting resolution!\n";
		return false;
	}
	
	if ((llt_.SetProfileConfig(CONTAINER)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting Container Mode!\n";
		return false;
	}
	
	if ((llt_.SetBufferCount(4)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting BufferCount!\n";
		return false;
	}
	
	if (llt_.SetFeature(FEATURE_FUNCTION_IDLETIME, idle_time_) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting uiIdleTime!\n";
		return false;
	}
	
	if (llt_.SetFeature(FEATURE_FUNCTION_SHUTTERTIME, shutter_time_) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting uiShutterTime!\n";
		return false;
	}
	
	if (llt_.SetFeature(FEATURE_FUNCTION_TRIGGER, 0x00000000) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting trigger!\n";
		return false;
	}
	
	//Setting High Voltage mode
	if (llt_.SetFeature(0xfffff0f008c0, 0x82000820) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting High Voltage!\n";
		return false;
	}
	
	int iRetValue;
	
	if((iRetValue = llt_.SetPacketSize(scanner_resolution)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error during SetPacketSize\n" << iRetValue;
		return false;;
	}
	
		
	
	/* Register Callbacks for program handling */
	if ((llt_.RegisterBufferCallback((gpointer)&Scanner26xx::new_profile_callback_wrapper,this)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while registering buffer callback!\n";
		return false;;
	}
	
	if ((llt_.RegisterControlLostCallback((gpointer)&Scanner26xx::control_lost_callback_wrapper,this)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while registering control lost callback!\n";
		return false;;
	}
	
	return true;
}


bool Scanner26xx::disconnect()
{
	if(!connected_)
	{
		return true;
	}
	llt_.Disconnect();
	connected_ = false;
	return true;
}
void Scanner26xx::control_lost_callback_wrapper(ArvGvDevice *gv_device, gpointer user_data)
{
	((Scanner26xx*)user_data)->control_lost_callback(gv_device);
}

void Scanner26xx::new_profile_callback_wrapper (const void * data, size_t data_size, gpointer user_data)
{
	((Scanner26xx*)user_data)->new_profile_callback(data, data_size);
}

void Scanner26xx::control_lost_callback(ArvGvDevice *gv_device)
{
	connected_ = false;
	std::cout << "Connection to scanner lost! Trying to reconnect!" << std::endl;
	bool was_scanning = scanning_;
	reconnect();
	if(was_scanning)
	{
		startScanning();
	}
	
}
void DisplayTimestamp (unsigned char * uiTimestamp)
{
	double dShutterOpen, dShutterClose;
	unsigned int uiProfileCount;
	
	Timestamp2TimeAndCount(&uiTimestamp[0], &dShutterOpen, &dShutterClose, &uiProfileCount);
	
	std::cout.precision(8);
	std::cout << "Profile Count: " << uiProfileCount << " ShutterOpen: " << dShutterOpen << " ShutterClose: " << dShutterClose << "\n";
	std::cout.precision(6);
}
void Scanner26xx::new_profile_callback (const void * data, size_t data_size)
{
	{
		boost::mutex::scoped_lock lock(mutex_);
		if(data != NULL && data_size == scanner_resolution*fieldCount_* container_size_*2)
		{
			memcpy(&container_buffer_[0],data,data_size);
		}
		
		unsigned int profile_counter;
		double shutter_open;
		double shutter_close;
		Timestamp2TimeAndCount(container_buffer_[container_buffer_.size()-1].timestamp, &shutter_open, &shutter_close, &profile_counter);
		if(need_time_sync_)
		{
			time_sync_->sync_time(profile_counter,shutter_open,shutter_close);
			need_time_sync_ = false;
		}
		for(int i = 0; i < container_buffer_.size(); ++i)
		{
			ScanProfileConvertedPtr profile (new ScanProfileConverted);
			Timestamp2TimeAndCount(container_buffer_[i].timestamp, &profile->shutter_open, &profile->shutter_close, &profile->profile_counter);
			//DisplayTimestamp(container_buffer_[i].timestamp);
			for(int j = 0; j < scanner_resolution; ++j)
			{
				if(container_buffer_[i].z[j] != 0)
				{
					double x = ((container_buffer_[i].x[j] - (guint16)32768) * llt_.appData.scaling) / 1000.0; //in meter
					double z = ((container_buffer_[i].z[j] - (guint16)32768) * llt_.appData.scaling + llt_.appData.offset) / 1000.0; //in meter
					profile->x.push_back(x);
					profile->z.push_back(z);
				}
			}
			
			profile_queue_.push(profile);
			
		}
	}
	notifyee_->notify();
	
}

bool Scanner26xx::hasNewData()
{
	boost::mutex::scoped_lock lock(mutex_);
	bool ret = !profile_queue_.empty();
	return ret;
}

ScanProfileConvertedPtr Scanner26xx::getData()
{
	boost::mutex::scoped_lock lock(mutex_);
	ScanProfileConvertedPtr ptr;
	if(profile_queue_.empty())
	{
		return ptr;
	}
	
	ptr = profile_queue_.front();
	profile_queue_.pop();
	return ptr;	
}


bool Scanner26xx::startScanning()
{
	need_time_sync_ = true;
	if(!connected_)
	{
		return false;
	}
	if(scanning_)
	{
		return true;
	}
	int iRetValue;
	
	
	
	unsigned int dwInquiry;
	
	//Bitfeld=Round(Log2(Auflösung)) for the resolution bitfield for the container
	//double dTempLog = 1.0/log(2.0);
	//unsigned int dwResolutionBitField = (unsigned int)floor((log((double)m_uiResolution)*dTempLog)+0.5);
	

	if((iRetValue = llt_.GetFeature(INQUIRY_FUNCTION_REARRANGEMENT_PROFILE, &dwInquiry)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error during GetFeature";
		return false;
	}
	
	if((dwInquiry & 0x80000000) == 0)
	{
		std::cout << "\nThe connected scanCONTROL don't support the container mode\n\n";
		return false;
	}
	
	//Extract Z
	//Extract X
	//calculation for the points per profile = 9 for 640
	//Extract only 1th reflection
	//Extract timestamp in extra field
	
	if((iRetValue = llt_.SetFeature(FEATURE_FUNCTION_REARRANGEMENT_PROFILE, 0x00120c03 | 9<<12)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error during SetFeature(FEATURE_FUNCTION_REARRANGEMENT_PROFILE)";
		return false;
	}
	
	std::cout << "Set profile container size\n";
	if((iRetValue = llt_.SetProfileContainerSize(scanner_resolution*fieldCount_, container_size_)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error during SetProfileContainerSize";
		return false;
	}
	
	
	setMeasuringField(field_.x_start,field_.x_size,field_.z_start,field_.z_size);
	
		
	// Setup transfer of multiple profiles
	if ((iRetValue = llt_.TransferProfiles(NORMAL_CONTAINER_MODE, true)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error in profile transfer!\n";
		return false;
	}
	scanning_ = true;
	return true;
}

bool Scanner26xx::stopScanning()
{
	if(!connected_)
	{
		scanning_ = false;
		return true;
	}
	if(!scanning_)
	{
		return true;
	}
	if ((llt_.TransferProfiles(NORMAL_CONTAINER_MODE, false)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while stopping transmission!\n";
		return false;
	}
	scanning_ = false;
	return true;
	
}




Scanner26xx::Scanner26xx(TimeSync* time_sync,Notifyee* notifyee,unsigned int shutter_time, unsigned int idle_time, unsigned int container_size, MeasurementField field,std::string serial_number, std::string path_to_device_properties) 
: time_sync_(time_sync), notifyee_(notifyee), shutter_time_(shutter_time), idle_time_(idle_time), container_size_(container_size), field_(field),serial_number_(serial_number)
{
	connected_ = false;
	scanning_  = false;
	fieldCount_ = 3;
	container_buffer_.resize(container_size_);
	path_to_device_properties_ = path_to_device_properties;
	connect();
	if(connected_)
	{
		if(!initialise())
                {
                    disconnect();
                    return;
                }
		if(!setLaserPower(true))
		{
			disconnect();
			return;
		}
			
	}
}
Scanner26xx::~Scanner26xx()
{
	if(connected_)
	{
		if(scanning_)
		{
			stopScanning();
		}
		setLaserPower(false);
		disconnect();
	}		
}

bool Scanner26xx::reconnect()
{
	if(connected_)
	{
		if(scanning_)
		{
			stopScanning();
		}
		setLaserPower(false);
		disconnect();
	}		
	connect();
	if(connected_)
	{
		if(!initialise())
		{
			return false;
		}
		if(!setLaserPower(true))
		{
			disconnect();
			return false;
		}
		
	}
	else
	{
		return false;
	}
	return true;
}


bool Scanner26xx::setLaserPower(bool on)
{
	guint32 value = on ? 0x82000002 : 0x82000000;
	
	if (llt_.SetFeature(FEATURE_FUNCTION_LASERPOWER, value) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting trigger!\n";
		return false;
	}
	
	return true;
}

// Schreibkommando für seq. Register
void Scanner26xx::WriteCommand(unsigned int command, unsigned int data)
{
	static int toggle = 0;
	llt_.SetFeature(FEATURE_FUNCTION_SHARPNESS,
					 (unsigned int)(command << 9) + (toggle << 8) + data);
	toggle = toggle ? 0 : 1;
}
// Schreibe Wert auf Registerposition
void Scanner26xx::WriteValue2Register(unsigned short value)
{
	WriteCommand(1, (unsigned int)(value/256));
	WriteCommand(1, (unsigned int)(value%256));
}


void Scanner26xx::setMeasuringField(ushort x_start, ushort x_size, ushort z_start, ushort z_size)
{

	// Aktiviere freies Messfeld
	llt_.SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, 0x82000800);
	// Setze die gewünschte Messfeldgröße

	WriteCommand(0, 0); // Resetkommando
	WriteCommand(0, 0);
	WriteCommand(2, 8);
	WriteValue2Register(z_start);
	WriteValue2Register(z_size);
	WriteValue2Register(x_start);
	WriteValue2Register(x_size);
	WriteCommand(0, 0); // Beende Schreibvorgang
}


