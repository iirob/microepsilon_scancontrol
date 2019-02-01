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

#include "microepsilon_scancontrol.h"

namespace microepsilon_scancontrol
{
bool Scanner::connect()
{
  if (connected_)
  {
    return true;
  }

  std::vector<char *> vcInterfaces(5);
  std::vector<unsigned int> vuiResolutions(10);
  unsigned int uiInterfaceCount = 0;

  int activeDevice = 0;

  int iRetValue = CInterfaceLLT::GetDeviceInterfaces(&vcInterfaces[0], vcInterfaces.size());
  if (iRetValue == ERROR_GETDEVINTERFACE_REQUEST_COUNT)
  {
    std::cout << "There are more than " << vcInterfaces.size() << " scanCONTROL connected \n";
    uiInterfaceCount = vcInterfaces.size();
  }
  else if (iRetValue < 1)
  {
    std::cout << "A error occured during searching for connected scanCONTROL \n";
    uiInterfaceCount = 0;
    return false;
  }
  else
  {
    uiInterfaceCount = iRetValue;
    if (uiInterfaceCount == 0)
      std::cout << "There is no scanCONTROL connected \n";
    else if (uiInterfaceCount == 1)
      std::cout << "There is 1 scanCONTROL connected \n";
    else
      std::cout << "There are " << uiInterfaceCount << " scanCONTROL connected \n";
    bool foundSN = false;
    for (int i = 0; i < uiInterfaceCount; ++i)
    {
      std::cout << vcInterfaces[i] << std::endl;
      std::string tempStr = vcInterfaces[i];
      if (serial_number_.size() != 0 &&
          tempStr.compare(tempStr.size() - serial_number_.size(), serial_number_.size(), serial_number_) == 0)
      {
        std::cout << "Found Device with serial number: " << serial_number_ << std::endl;
        foundSN = true;
        activeDevice = i;
        break;
      }
    }
    if (!foundSN && serial_number_.size() != 0)
    {
      std::cout << "Could not find device with S/N: " << serial_number_ << ". Using first device in list." << std::endl;
    }
  }

  if ((iRetValue = llt_.SetPathDeviceProperties(path_to_device_properties_.c_str())) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error setting device ID path\nExit program...\n";
    return false;
  }

  std::cout << "Connecting to " << vcInterfaces[activeDevice] << std::endl;
  if ((llt_.SetDeviceInterface(vcInterfaces[activeDevice])) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting dev id " << iRetValue << "!\n";
    return false;
  }

  /* Connect to sensor */
  if ((iRetValue = llt_.Connect()) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
    return false;
  }

  if ((iRetValue = llt_.GetLLTType(&type_)) < GENERAL_FUNCTION_OK)
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

  if (type_ == scanCONTROL27xx_xxx)
  {
    std::cout << "The scanCONTROL is a scanCONTROL27xx\n";
  }
  else if (type_ == scanCONTROL26xx_xxx)
  {
    std::cout << "The scanCONTROL is a scanCONTROL26xx\n";
  }
  else if (type_ == scanCONTROL29xx_xxx)
  {
    std::cout << "The scanCONTROL is a scanCONTROL29xx\n";
  }
  else
  {
    std::cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n";
  }
  CInterfaceLLT::GetScalingAndOffsetByType(type_, &scaling_, &offset_);
  connected_ = true;
  return true;
}

bool Scanner::initialise()
{
  if (!connected_)
  {
    return false;
  }

  // set to default config
  llt_.ReadWriteUserModes(false, 0);

  if (llt_.SetResolution(SCANNER_RESOLUTION) < GENERAL_FUNCTION_OK)
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

  // Setting High Voltage mode
  if (llt_.SetFeature(0xf0f008c0, 0x82000820) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting High Voltage!\n";
    return false;
  }

  int iRetValue;

  if ((iRetValue = llt_.SetPacketSize(1024)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error during SetPacketSize\n" << iRetValue;
    return false;
    ;
  }

  /* Register Callbacks for program handling */
  if ((llt_.RegisterBufferCallback((gpointer)&Scanner::new_profile_callback_wrapper, this)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while registering buffer callback!\n";
    return false;
    ;
  }

  if ((llt_.RegisterControlLostCallback((gpointer)&Scanner::control_lost_callback_wrapper, this)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while registering control lost callback!\n";
    return false;
    ;
  }

  return true;
}

void Scanner::reconfigure(ScannerConfig config, bool restart)
{
  config_ = config;
  int shutter_time = 100 * config_.shutter_time;
  int idle_time = 100000.0 / config_.frequency - shutter_time;
  unsigned int shutter_time_flags = config_.auto_shutter ? shutter_time | SHUTTER_AUTOMATIC : shutter_time;
  if (llt_.SetFeature(FEATURE_FUNCTION_SHUTTERTIME, shutter_time_flags) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting uiShutterTime!\n";
  }

  if (llt_.SetFeature(FEATURE_FUNCTION_IDLETIME, idle_time) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting uiIdleTime!\n";
  }

  // Laser Power and Pulse
  int laser_value = config_.laser_pulse ? config_.laser_power | LASER_PULSMODE : config_.laser_power;
  if (llt_.SetFeature(FEATURE_FUNCTION_LASERPOWER, laser_value) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting trigger!\n";
  }

  // Meassurement field
  MeasurementField field(config_.field_left, config_.field_right, config_.field_far, config_.field_near);
  llt_.SetFeature(FEATURE_FUNCTION_MEASURINGFIELD, MEASFIELD_ACTIVATE_FREE);
  llt_.SetFreeMeasuringField(field.x_start, field.x_size, field.z_start, field.z_size);

  // Peak filter
  llt_.SetPeakFilter(config_.peak_width_min, config_.peak_width_max, config_.peak_intensity_min,
                     config_.peak_intensity_max);

  // Threshold
  int threshold_value = config_.dynamic_threshold << 24 | config_.video_filter << 11 |
                        config_.ambient_suppresion << 10 | config_.threshold;
  if (llt_.SetFeature(FEATURE_FUNCTION_THRESHOLD, threshold_value) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting FEATURE_FUNCTION_THRESHOLD!\n";
  }
  // Processing
  int processing_value = PROC_HIGH_RESOLUTION | PROC_CALIBRATION | config_.reflection << 2 | config_.flip_z << 6 |
                         config_.flip_x << 7 | config_.late_auto_shutter << 8 | config_.shutter_alignment << 9 |
                         config_.shutter_algorithm << 11;
  if (llt_.SetFeature(FEATURE_FUNCTION_PROCESSING_PROFILEDATA, processing_value) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting FEATURE_FUNCTION_PROCESSING_PROFILEDATA!\n";
  }

  // Resampling
  int resample_value = config_.interpolate << 11 | config_.resample_all << 10 | config_.resample << 4 |
                       config_.median_filter << 2 | config_.average_filter;
  if (llt_.SetFeature(FEATURE_FUNCTION_PROFILE_FILTER, resample_value) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting FEATURE_FUNCTION_PROFILE_FILTER!\n";
  }

  if(restart)
  {
    stopScanning();
    startScanning();
  }
}

bool Scanner::disconnect()
{
  if (!connected_)
  {
    return true;
  }
  llt_.Disconnect();
  connected_ = false;
  return true;
}
void Scanner::control_lost_callback_wrapper(ArvGvDevice *gv_device, gpointer user_data)
{
  ((Scanner *)user_data)->control_lost_callback(gv_device);
}

void Scanner::new_profile_callback_wrapper(const void *data, size_t data_size, gpointer user_data)
{
  ((Scanner *)user_data)->new_profile_callback(data, data_size);
}

void Scanner::control_lost_callback(ArvGvDevice *gv_device)
{
  connected_ = false;
  std::cout << "Connection to scanner lost! Trying to reconnect!" << std::endl;
  bool was_scanning = scanning_;
  reconnect();
  if (was_scanning)
  {
    startScanning();
  }
}
void DisplayTimestamp(unsigned char *uiTimestamp)
{
  double dShutterOpen, dShutterClose;
  unsigned int uiProfileCount;
  gushort enc_times;

  CInterfaceLLT::Timestamp2TimeAndCount(&uiTimestamp[0], &dShutterOpen, &dShutterClose, &uiProfileCount, &enc_times);

  std::cout.precision(8);
  std::cout << "Profile Count: " << uiProfileCount << " ShutterOpen: " << dShutterOpen
            << " ShutterClose: " << dShutterClose << " ShutterTime: " << (dShutterClose - dShutterOpen) * 1000.0 << "\n";
  std::cout.precision(6);
}

void Scanner::new_profile_callback(const void *data, size_t data_size)
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (data != NULL && data_size == SCANNER_RESOLUTION * fieldCount_ * config_.container_size * 2)
    {
      memcpy(&container_buffer_[0], data, data_size);
    }

    unsigned int profile_counter;
    double shutter_open;
    double shutter_close;
    CInterfaceLLT::Timestamp2TimeAndCount(container_buffer_[container_buffer_.size() - 1].timestamp, &shutter_open,
                                          &shutter_close, &profile_counter, &enc_times_);
    if (need_time_sync_)
    {
      time_sync_->sync_time(profile_counter, shutter_open, shutter_close);
      need_time_sync_ = false;
    }
    for (int i = 0; i < container_buffer_.size(); ++i)
    {
      ScanProfileConvertedPtr profile(new ScanProfileConverted);
      CInterfaceLLT::Timestamp2TimeAndCount(container_buffer_[i].timestamp, &profile->shutter_open,
                                            &profile->shutter_close, &profile->profile_counter, &enc_times_);
      // DisplayTimestamp(container_buffer_[i].timestamp);
      for (int j = 0; j < SCANNER_RESOLUTION; ++j)
      {
        double x = ((container_buffer_[i].x[j] - (guint16)32768) * scaling_) / 1000.0;            // in meter
        double z = ((container_buffer_[i].z[j] - (guint16)32768) * scaling_ + offset_) / 1000.0;  // in meter

        if (container_buffer_[i].z[j] == 0)
        {
          z = std::numeric_limits<double>::quiet_NaN();
        }
        if (!config_.dense || container_buffer_[i].z[j] != 0)
        {
          profile->x.push_back(x);
          profile->z.push_back(z);
          profile->intensity.push_back(container_buffer_[i].intensity[j]);
          }
      }

      profile_queue_.push(profile);
    }
  }
  notifyee_->notify();
}

bool Scanner::hasNewData()
{
  boost::mutex::scoped_lock lock(mutex_);
  bool ret = !profile_queue_.empty();
  return ret;
}

ScanProfileConvertedPtr Scanner::getData()
{
  boost::mutex::scoped_lock lock(mutex_);
  ScanProfileConvertedPtr ptr;
  if (profile_queue_.empty())
  {
    return ptr;
  }

  ptr = profile_queue_.front();
  profile_queue_.pop();
  return ptr;
}

bool Scanner::startScanning()
{
  need_time_sync_ = true;
  if (!connected_)
  {
    return false;
  }
  if (scanning_)
  {
    return true;
  }
  int iRetValue;

  unsigned int dwInquiry;

  // Bitfeld=Round(Log2(Auflösung)) for the resolution bitfield for the container
  // double dTempLog = 1.0/log(2.0);
  // unsigned int dwResolutionBitField = (unsigned int)floor((log((double)m_uiResolution)*dTempLog)+0.5);

  if ((iRetValue = llt_.GetFeature(INQUIRY_FUNCTION_REARRANGEMENT_PROFILE, &dwInquiry)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error during GetFeature";
    return false;
  }

  if ((dwInquiry & 0x80000000) == 0)
  {
    std::cout << "\nThe connected scanCONTROL don't support the container mode\n\n";
    return false;
  }

  // Extract Z
  // Extract X
  // calculation for the points per profile = 9 for 640
  // Extract only 1th reflection
  // Extract timestamp in extra field

  int rearrengement_value = CONTAINER_DATA_X | CONTAINER_DATA_Z | CONTAINER_DATA_INTENS | CONTAINER_DATA_TS |
                            CONTAINER_DATA_EMPTYFIELD4TS | CONTAINER_STRIPE_1 | CONTAINER_DATA_LSBF | 9 << 12;
  if ((iRetValue = llt_.SetFeature(FEATURE_FUNCTION_REARRANGEMENT_PROFILE, rearrengement_value)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error during SetFeature(FEATURE_FUNCTION_REARRANGEMENT_PROFILE)";
    return false;
  }

  std::cout << "Set profile container size\n";
  container_buffer_.resize(config_.container_size);
  if ((iRetValue = llt_.SetProfileContainerSize(SCANNER_RESOLUTION * fieldCount_, config_.container_size)) <
      GENERAL_FUNCTION_OK)
  {
    std::cout << "Error during SetProfileContainerSize";
    return false;
  }

  // setMeasuringField(field_.x_start, field_.x_size, field_.z_start, field_.z_size);

  // Setup transfer of multiple profiles
  if ((iRetValue = llt_.TransferProfiles(NORMAL_CONTAINER_MODE, true)) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error in profile transfer!\n";
    return false;
  }
  scanning_ = true;
  return true;
}

bool Scanner::stopScanning()
{
  if (!connected_)
  {
    scanning_ = false;
    return true;
  }
  if (!scanning_)
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

Scanner::Scanner(TimeSync *time_sync, Notifyee *notifyee, unsigned int container_size, std::string serial_number,
                 std::string path_to_device_properties)
  : time_sync_(time_sync), notifyee_(notifyee), serial_number_(serial_number)
{
  connected_ = false;
  scanning_ = false;
  fieldCount_ = 4;
  config_.container_size = container_size;
  container_buffer_.resize(config_.container_size);
  path_to_device_properties_ = path_to_device_properties;
  path_to_device_properties_ += "/device_properties.dat";
  connect();
  if (connected_)
  {
    if (!initialise())
    {
      disconnect();
      return;
    }
    if (!setLaserPower(true))
    {
      disconnect();
      return;
    }
  }
}
Scanner::~Scanner()
{
  if (connected_)
  {
    if (scanning_)
    {
      stopScanning();
    }
    setLaserPower(false);
    disconnect();
  }
}

bool Scanner::reconnect()
{
  if (connected_)
  {
    if (scanning_)
    {
      stopScanning();
    }
    setLaserPower(false);
    disconnect();
  }
  connect();
  if (connected_)
  {
    if (!initialise())
    {
      return false;
    }
    if (!setLaserPower(true))
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

bool Scanner::setLaserPower(bool on)
{
  int laser_value = config_.laser_pulse ? config_.laser_power | LASER_PULSMODE : config_.laser_power;
  guint32 value = on ? laser_value : 0x82000000;

  if (llt_.SetFeature(FEATURE_FUNCTION_LASERPOWER, value) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting trigger!\n";
    return false;
  }

  return true;
}

}  // namespace microepsilon_scancontrol
