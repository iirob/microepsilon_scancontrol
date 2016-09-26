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
	}
	
	if (uiInterfaceCount == 0)
		std::cout << "There is no scanCONTROL connected \n";
	else if (uiInterfaceCount == 1)
		std::cout << "There is 1 scanCONTROL connected \n";
	else
		std::cout << "There are " << uiInterfaceCount << " scanCONTROL connected \n";
	
	
	
	if ((iRetValue = SetPathtoDeviceProperties("./device_properties.dat")) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error setting device ID path\nExit program...\n";
		return false;
	}
	
	
	
	if ((llt_.SetDeviceInterface(vcInterfaces[0])) < GENERAL_FUNCTION_OK)
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
	if (llt_.SetResolution(scanner_resolution) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting resolution!\n";
		return false;
	}

	llt_.ReadWriteUserModes(false,0);
	
	if ((llt_.SetProfileConfig(CONTAINER)) < GENERAL_FUNCTION_OK)
	{
		std::cout << "Error while setting Container Mode!\n";
		return false;
	}
	
	if ((llt_.SetBufferCount(10)) < GENERAL_FUNCTION_OK)
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
	mutex_.lock();
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
	mutex_.unlock();
}

bool Scanner26xx::hasNewData()
{
	mutex_.lock();
	bool ret = !profile_queue_.empty();
	mutex_.unlock();
	return ret;
}

ScanProfileConvertedPtr Scanner26xx::getData()
{
	mutex_.lock();
	ScanProfileConvertedPtr ptr;
	if(profile_queue_.empty())
	{
		mutex_.unlock();
		return ptr;
	}
	
	ptr = profile_queue_.front();
	profile_queue_.pop();
	mutex_.unlock();
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




Scanner26xx::Scanner26xx(TimeSync* time_sync) : time_sync_(time_sync)
{
	connected_ = false;
	scanning_  = false;
	container_size_ = 1;
	idle_time_ = 100;
	shutter_time_ = 400;
	fieldCount_ = 3;
	container_buffer_.resize(container_size_);
	
	connect();
	if(connected_)
	{
		initialise();	
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

