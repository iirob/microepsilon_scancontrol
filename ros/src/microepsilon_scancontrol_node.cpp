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

#include "ros/ros.h"
#include "microepsilon_scancontrol.h"

#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>

namespace microepsilon_scancontrol
{
double average(double a, double b)
{
  return (a + b) / 2.0;
}

class ScannerNode : public TimeSync, Notifyee
{
public:
  ScannerNode(unsigned int shutter_time, unsigned int idle_time, unsigned int container_size,
                  MeasurementField field, double lag_compensation, std::string topic, std::string frame,
                  std::string serial_number, std::string path_to_device_properties);

  void publish();
  bool startScanning();
  bool stopScanning();
  bool reconnect();
  virtual void sync_time(unsigned int profile_counter, double shutter_open, double shutter_close);
  virtual void notify();

private:
  void initialiseMessage();
  bool laser_on(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool laser_off(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  ros::Publisher scan_pub_;
  ros::Publisher meassured_z_pub_;
  ros::ServiceServer laser_on_, laser_off_;

  ros::NodeHandle nh_;
  // laser data
  Scanner26xx laser_;
  int last_second_;
  double lag_compensation_;
  // published data
  sensor_msgs::PointCloud2 cloud_msg_;
  // parameters
  ros::Duration shutter_close_sync_;
  std::string frame_;
  bool publishing_;
};

ScannerNode::ScannerNode(unsigned int shutter_time, unsigned int idle_time, unsigned int container_size,
                                 MeasurementField field, double lag_compensation, std::string topic, std::string frame,
                                 std::string serial_number, std::string path_to_device_properties)
  : laser_(this, this, shutter_time, idle_time, container_size, field, serial_number, path_to_device_properties)
  , lag_compensation_(lag_compensation)
  , frame_(frame)
{
  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 500);
  meassured_z_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("meassured_z", 50);
  laser_off_ = nh_.advertiseService("laser_off", &Scanner26xxNode::laser_off, this);
  laser_on_ = nh_.advertiseService("laser_on", &Scanner26xxNode::laser_on, this);
  publishing_ = true;
  initialiseMessage();
  ROS_INFO("Connecting to Laser");
}

void ScannerNode::sync_time(unsigned int profile_counter, double shutter_open, double shutter_close)
{
  ROS_DEBUG("New Timestamp: %d %9f", profile_counter, average(shutter_open, shutter_close));
  shutter_close_sync_ =
      ros::Time::now() - ros::Time(average(shutter_open, shutter_close)) - ros::Duration(lag_compensation_);
  last_second_ = 0;
}

void ScannerNode::notify()
{
  publish();
}

bool ScannerNode::laser_off(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  publishing_ = false;
  return laser_.setLaserPower(false);
}
bool ScannerNode::laser_on(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  publishing_ = true;
  return laser_.setLaserPower(true);
}

void ScannerNode::initialiseMessage()
{
  cloud_msg_.header.frame_id = frame_;
  cloud_msg_.is_bigendian = false;
  cloud_msg_.is_dense = true;
  cloud_msg_.height = 1;
  cloud_msg_.width = 640;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.reserve(640);
}

void ScannerNode::publish()
{
  while (laser_.hasNewData())
  {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
    ScanProfileConvertedPtr data = laser_.getData();
    ros::Time profile_time(average(data->shutter_open, data->shutter_close));
    if (profile_time.toSec() - last_second_ < 0)
    {
      shutter_close_sync_ += ros::Duration(128);
    }
    last_second_ = profile_time.toSec();
    if (publishing_)
    {
      cloud_msg_.header.stamp = profile_time + shutter_close_sync_;
      ++cloud_msg_.header.seq;
      ROS_DEBUG_STREAM(profile_time << " " << cloud_msg_.header.stamp);
      sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
      modifier.resize(data->x.size());
      static bool firstrun = true;
      if (firstrun)
      {
        ROS_INFO_STREAM("Points per profile: " << data->x.size());
        firstrun = false;
      }
      for (int i = 0; i < data->x.size(); ++i, ++iter_x, ++iter_z, ++iter_y)
      {
        *iter_x = data->x[i];
        *iter_z = data->z[i];
        *iter_y = 0.0;
      }
      scan_pub_.publish(cloud_msg_);
      std_msgs::Float32MultiArray meassured_z;
      if (data->z.size() > 0)
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

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_26xx_node");

  ros::NodeHandle nh_private("~");

  int shutter_time;
  int idle_time;
  int container_size;
  double lag_compensation;
  std::string topic, frame, serial_number, path_to_device_properties;
  double field_left, field_right, field_far, field_near;
  if (!nh_private.getParam("shutter_time", shutter_time))
  {
    ROS_ERROR("You have to specify parameter shutter_time!");
    return -1;
  }
  if (!nh_private.getParam("idle_time", idle_time))
  {
    ROS_ERROR("You have to specify parameter idle_time!");
    return -1;
  }
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
  if (!nh_private.getParam("field_left", field_left))
  {
    field_left = 0.0;
  }
  if (!nh_private.getParam("field_right", field_right))
  {
    field_right = 0.0;
  }
  if (!nh_private.getParam("field_far", field_far))
  {
    field_far = 0.0;
  }
  if (!nh_private.getParam("field_near", field_near))
  {
    field_near = 0.0;
  }
  if (!nh_private.getParam("lag_compensation", lag_compensation))
  {
    lag_compensation = 0.0;
  }
  ROS_INFO("Shutter Time: %.2fms Idle Time: %.2fms Frequency: %.2fHz", shutter_time / 100.0, idle_time / 100.0,
           100000.0 / (shutter_time + idle_time));
  ROS_INFO("Profiles for each Container: %d", container_size);
  ROS_INFO("Lag compensation: %.3fms", lag_compensation * 1000);

  field_left = fmin(fmax(field_left, 0.0), 1.0);
  field_right = fmin(fmax(field_right, 0.0), 1.0);
  field_far = fmin(fmax(field_far, 0.0), 1.0);
  field_near = fmin(fmax(field_near, 0.0), 1.0);
  MeasurementField field(field_left, field_right, field_far, field_near);
  ScannerNode scanner(shutter_time, idle_time, container_size, field, lag_compensation, topic, frame, serial_number,
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

}  // namespace microepsilon_scancontrol
