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
#include "ros/ros.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include "sensor_msgs/PointCloud2.h"

#include <dynamic_reconfigure/server.h>
#include <microepsilon_scancontrol/ScanControlConfig.h>

namespace microepsilon_scancontrol
{
double average(double a, double b)
{
  return (a + b) / 2.0;
}

class ScannerNode : public TimeSync, Notifyee
{
public:
  ScannerNode(unsigned int container_size, std::string topic, std::string frame,
              std::string serial_number, std::string path_to_device_properties);

  void publish();
  bool startScanning();
  bool stopScanning();
  bool reconnect();
  virtual void sync_time(unsigned int profile_counter, double shutter_open, double shutter_close);
  virtual void notify();

private:
  void initialiseMessage();
  bool laser_on(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool laser_off(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void dynrec_callback(microepsilon_scancontrol::ScanControlConfig &config, uint32_t level);

  ros::Publisher scan_pub_;
  ros::Publisher meassured_z_pub_;
  ros::ServiceServer laser_on_, laser_off_;

  ros::NodeHandle nh_;
  // laser data
  Scanner laser_;
  ScannerConfig laser_config_;
  dynamic_reconfigure::Server<microepsilon_scancontrol::ScanControlConfig> dynrec_server_;

  int last_second_;
  // published data
  sensor_msgs::PointCloud2 cloud_msg_;
  // parameters
  ros::Duration shutter_mid_sync_;
  std::string frame_;
  bool publishing_;
};

ScannerNode::ScannerNode(unsigned int container_size, std::string topic, std::string frame, std::string serial_number,
                         std::string path_to_device_properties)
    : laser_(this, this, container_size, serial_number, path_to_device_properties), frame_(frame)
{
  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 500);
  meassured_z_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("meassured_z", 50);
  laser_off_ = nh_.advertiseService("laser_off", &ScannerNode::laser_off, this);
  laser_on_ = nh_.advertiseService("laser_on", &ScannerNode::laser_on, this);
  publishing_ = true;
  initialiseMessage();
  dynrec_server_.setCallback(boost::bind(&ScannerNode::dynrec_callback, this, _1, _2));
  ROS_INFO("Connecting to Laser");
}

void ScannerNode::sync_time(unsigned int profile_counter, double shutter_open, double shutter_close)
{
  ROS_DEBUG("New Timestamp: %d %9f", profile_counter, shutter_close);
  shutter_mid_sync_ = ros::Time::now() - ros::Time(shutter_close);
  last_second_ = 0;
}

void ScannerNode::notify()
{
  publish();
}

bool ScannerNode::laser_off(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // publishing_ = false;
  return laser_.setLaserPower(false);
}
bool ScannerNode::laser_on(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // publishing_ = true;
  return laser_.setLaserPower(true);
}

void ScannerNode::initialiseMessage()
{
  cloud_msg_.header.frame_id = frame_;
  cloud_msg_.is_bigendian = false;
  cloud_msg_.is_dense = laser_config_.dense;
  cloud_msg_.height = 1;
  cloud_msg_.width = 640;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1,
                                sensor_msgs::PointField::UINT16);
  modifier.reserve(640);
}

void ScannerNode::publish()
{
  while (laser_.hasNewData())
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_intensity(cloud_msg_, "intensity");
    ScanProfileConvertedPtr data = laser_.getData();
    ros::Time profile_time(data->shutter_close);
    if (profile_time.toSec() - last_second_ < 0)
    {
      ROS_WARN_STREAM("Time overflow! profile time: " << profile_time.toSec() << " last_second: " << last_second_ << " shutter_mid_sync: " << shutter_mid_sync_);
      shutter_mid_sync_ += ros::Duration(128);
    }
    last_second_ = profile_time.toSec();
    if (publishing_)
    {
      cloud_msg_.header.stamp = profile_time + shutter_mid_sync_ - ros::Duration(laser_config_.lag_compensation);
      ++cloud_msg_.header.seq;
      cloud_msg_.is_dense = laser_config_.dense;
      cloud_msg_.header.frame_id = frame_;
      ROS_DEBUG_STREAM(profile_time << " " << cloud_msg_.header.stamp);
      sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
      modifier.resize(data->x.size());
      static bool firstrun = true;
      if (firstrun)
      {
        ROS_INFO_STREAM("Points per profile: " << data->x.size());
        firstrun = false;
      }
      for (int i = 0; i < data->x.size(); ++i, ++iter_x, ++iter_z, ++iter_y, ++iter_intensity)
      {
        *iter_x = data->x[i];
        *iter_z = data->z[i];
        *iter_y = 0.0;
        *iter_intensity = data->intensity[i];
      }
      scan_pub_.publish(cloud_msg_);
      std_msgs::Float32MultiArray meassured_z;
      if (laser_config_.dense && data->z.size() > 0)
      {
        meassured_z.data.push_back((float)data->z[0]);
        meassured_z.data.push_back((float)data->z[data->z.size() / 2]);
        meassured_z.data.push_back((float)data->z[data->z.size() - 1]);
        meassured_z_pub_.publish(meassured_z);
      }
    }
  }
}

bool ScannerNode::startScanning()
{
  return laser_.startScanning();
}

bool ScannerNode::stopScanning()
{
  return laser_.stopScanning();
}

bool ScannerNode::reconnect()
{
  laser_.reconnect();
}

void ScannerNode::dynrec_callback(microepsilon_scancontrol::ScanControlConfig &config, uint32_t level)
{
  laser_config_.auto_shutter = config.auto_shutter;
  laser_config_.shutter_time = config.shutter_time;
  laser_config_.frequency = config.frequency;
  laser_config_.field_left = config.field_left;
  laser_config_.field_right = config.field_right;
  laser_config_.field_far = config.field_far;
  laser_config_.field_near = config.field_near;
  laser_config_.dense = config.dense;
  laser_config_.laser_power = config.laser_power;
  laser_config_.laser_pulse = config.laser_pulse;
  laser_config_.lag_compensation = config.lag_compensation;
  laser_config_.reflection = config.reflection;
  laser_config_.flip_x = config.flip_x;
  laser_config_.flip_z = config.flip_z;
  laser_config_.late_auto_shutter = config.late_auto_shutter;
  laser_config_.shutter_alignment = config.shutter_alignment;
  laser_config_.shutter_algorithm = config.shutter_algorithm;
  laser_config_.peak_width_min = config.peak_width_min;
  laser_config_.peak_width_max = config.peak_width_max;
  laser_config_.peak_intensity_min = config.peak_intensity_min;
  laser_config_.peak_intensity_max = config.peak_intensity_max;
  laser_config_.threshold = config.threshold;
  laser_config_.dynamic_threshold = config.dynamic_threshold;
  laser_config_.video_filter = config.video_filter;
  laser_config_.ambient_suppresion = config.ambient_suppresion;
  laser_config_.average_filter = config.average_filter;
  laser_config_.median_filter = config.median_filter;
  laser_config_.resample = config.resample;
  laser_config_.resample_all = config.resample_all;
  laser_config_.interpolate = config.interpolate;
  laser_config_.container_size = config.container_size;
  frame_ = config.frame;
  laser_.reconfigure(laser_config_, level);
}

} // namespace microepsilon_scancontrol

//#######################
//#### main programm ####
int main(int argc, char **argv)
{
  ros::init(argc, argv, "microepsilon_scancontrol_node");

  ros::NodeHandle nh_private("~");

  int container_size;
  std::string topic, frame, serial_number, path_to_device_properties;

  if (!nh_private.getParam("container_size", container_size))
  {
    ROS_ERROR("You have to specify parameter container_size!");
    return -1;
  }
  if (!nh_private.getParam("frame", frame))
  {
    ROS_ERROR("You have to specify parameter frame!");
    return -1;
  }
  if (!nh_private.getParam("path_to_device_properties", path_to_device_properties))
  {
    ROS_ERROR("You have to specify parameter path_to_device_properties!");
    return -1;
  }
  if (!nh_private.getParam("topic", topic))
  {
    topic = "laser_scan";
  }
  if (!nh_private.getParam("serial_number", serial_number))
  {
    serial_number = "";
  }

  ROS_INFO("Profiles for each Container: %d", container_size);

  microepsilon_scancontrol::ScannerNode scanner(container_size, topic, frame, serial_number,
                                                path_to_device_properties);
  bool scanning = scanner.startScanning();
  while (!scanning && !ros::isShuttingDown())
  {
    ROS_ERROR("Couldn't start scanning. Reconnecting!");
    scanner.reconnect();
    scanning = scanner.startScanning();
  }
  ROS_INFO("Started scanning.");

  ros::spin();
  scanner.stopScanning();

  return 0;
}
