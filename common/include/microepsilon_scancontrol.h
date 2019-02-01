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

#ifndef _MICROEPSILON_SCANCONTROL_ROS_H_
#define _MICROEPSILON_SCANCONTROL_ROS_H_

#include <llt.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <queue>
#include <vector>

namespace microepsilon_scancontrol
{
const int SCANNER_RESOLUTION = 640;

class TimeSync
{
public:
  virtual void sync_time(unsigned int profile_counter, double shutter_open, double shutter_close) = 0;
};

class Notifyee
{
public:
  virtual void notify() = 0;
};

struct MeasurementField
{
  ushort x_start, x_size, z_start, z_size;

  MeasurementField()
  {
    x_start = z_start = 0;
    x_size = z_size = 65535;
  }

  // Each variable defines the part of the border that gets cut off. Min 0.0, Max 1.0
  MeasurementField(double left, double right, double far, double near)
  {
    assert(left >= 0.0);
    assert(right >= 0.0);
    assert(far >= 0.0);
    assert(near >= 0.0);
    assert(left + right <= 1.0);
    assert(far + near <= 1.0);
    ushort umax = 65535;
    z_start = near * umax;
    z_size = umax - near * umax - far * umax;
    x_start = left * umax;
    x_size = umax - left * umax - right * umax;
  }
  MeasurementField(ushort x_start, ushort x_size, ushort z_start, ushort z_size)
    : x_start(x_start), x_size(x_size), z_start(z_start), z_size(z_size)
  {
  }
};

struct ScanProfile
{
  guint16 z[SCANNER_RESOLUTION];
  guint16 x[SCANNER_RESOLUTION];
  guint16 intensity[SCANNER_RESOLUTION];
  guint16 padding[SCANNER_RESOLUTION - 8];
  unsigned char timestamp[16];
};

struct ScanProfileConverted
{
  std::vector<double> x;
  std::vector<double> z;
  std::vector<uint16_t> intensity;
  unsigned int profile_counter;
  double shutter_open;
  double shutter_close;
};
typedef boost::shared_ptr<ScanProfileConverted> ScanProfileConvertedPtr;

struct ScannerConfig
{
  bool auto_shutter;
  double shutter_time;
  double frequency;
  double field_left;
  double field_right;
  double field_far;
  double field_near;
  bool dense;
  int laser_power;
  bool laser_pulse;
  double lag_compensation;
  int reflection;
  bool flip_x;
  bool flip_z;
  bool late_auto_shutter;
  int shutter_alignment;
  bool shutter_algorithm;
  int peak_width_min;
  int peak_width_max;
  int peak_intensity_min;
  int peak_intensity_max;
  int threshold;
  bool dynamic_threshold;
  bool video_filter;
  bool ambient_suppresion;
  int average_filter;
  int median_filter;
  int resample;
  bool resample_all;
  bool interpolate;
  int container_size;
};

class Scanner
{
private:
  bool scanning_;
  bool connected_;
  bool need_time_sync_;

  ScannerConfig config_;
  boost::mutex mutex_;
  unsigned int fieldCount_;
  TimeSync* time_sync_;
  Notifyee* notifyee_;
  std::string serial_number_, path_to_device_properties_;

  CInterfaceLLT llt_;
  TScannerType type_;
  double scaling_, offset_;
  gushort enc_times_;

  std::vector<ScanProfile> container_buffer_;
  std::queue<ScanProfileConvertedPtr> profile_queue_;
  bool connect();
  bool disconnect();
  bool initialise();

  void control_lost_callback(ArvGvDevice* gv_device);
  void new_profile_callback(const void* data, size_t data_size);
  static void control_lost_callback_wrapper(ArvGvDevice* gv_device, gpointer user_data);
  static void new_profile_callback_wrapper(const void* data, size_t data_size, gpointer user_data);

public:
  Scanner(TimeSync* time_sync, Notifyee* notifyee, unsigned int container_size, std::string serial_number,
          std::string path_to_device_properties);
  ~Scanner();

  bool reconnect();

  bool startScanning();
  bool stopScanning();
  bool setLaserPower(bool on);
  bool hasNewData();
  void reconfigure(ScannerConfig config, bool restart);
  ScanProfileConvertedPtr getData();
};

}  // namespace microepsilon_scancontrol
#endif
