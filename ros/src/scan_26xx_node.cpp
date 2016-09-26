#include "ros/ros.h"
#include "scanner26xx.h"
#include <signal.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>

sig_atomic_t volatile g_request_shutdown = 0;
void mySigIntHandler(int sig)
{
	g_request_shutdown = 1;
}

class Scanner26xxNode : public TimeSync
{
public:
	
	Scanner26xxNode();
	
	void publish();
	bool startScanning();
	bool stopScanning();
	bool reconnect();
	void sync_time(unsigned int profile_counter,double shutter_open,double shutter_close);
	
	
private:
	
	void initialiseMessage();
	
	ros::Publisher scan_pub_;
	ros::NodeHandle nh_;
	ros::Rate rate_;
	// laser data
	Scanner26xx laser_;
	
	// published data
	sensor_msgs::PointCloud2 cloud_msg_;
	// parameters
	ros::Time shutter_close_sync_;
};

Scanner26xxNode::Scanner26xxNode() :rate_(1000), laser_(this)
{
	scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_scan",500);
	initialiseMessage();
	ROS_INFO("Connecting to Laser");
}

void Scanner26xxNode::sync_time(unsigned int profile_counter, double shutter_open, double shutter_close)
{
	ROS_DEBUG("New Timestamp: %d %9f",profile_counter,shutter_close);
	shutter_close_sync_ = ros::Time(shutter_close);
}


void Scanner26xxNode::initialiseMessage()
{
	cloud_msg_.header.frame_id = "/scanner_link";
	cloud_msg_.is_bigendian = false;
	cloud_msg_.is_dense = true;
	cloud_msg_.height = 1;
	cloud_msg_.width = 640;
	sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
	modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
								  "y", 1, sensor_msgs::PointField::FLOAT32,
							   "z", 1, sensor_msgs::PointField::FLOAT32);
// 	scan_msg.fields.resize(2);
// 	scan_msg.fields[0].name = "x";
// 	scan_msg.fields[1].name = "z";
// 	scan_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT64;
// 	scan_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT64;
	modifier.reserve(640);
	
}

void Scanner26xxNode::publish()
{
	
	if(laser_.hasNewData())
	{
		sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
		sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
		ScanProfileConvertedPtr data = laser_.getData();
		ros::Time profile_time(data->shutter_close);
		ros::Duration time_diff = profile_time - shutter_close_sync_;
		if(time_diff < ros::Duration(0))
		{
			time_diff += ros::Duration(128);
		}
		cloud_msg_.header.stamp = ros::Time::now() + time_diff;
		++cloud_msg_.header.seq;
		sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
		modifier.resize(data->x.size());
		for(int i = 0; i < data->x.size(); ++i, ++iter_x, ++iter_z, ++iter_y)
		{
			*iter_x = data->x[i];
			*iter_z = data->z[i];
			*iter_y = 0.0;
		}
		scan_pub_.publish(cloud_msg_);
// 		sensor_msgs::PointCloud2Iterator<float> iter2_x(cloud_msg_, "x");
// 		sensor_msgs::PointCloud2Iterator<float> iter2_z(cloud_msg_, "z");
// 		sensor_msgs::PointCloud2Iterator<float> iter2_y(cloud_msg_, "y");
// 		for(int i = 0; i < data->x.size(); ++i, ++iter2_x, ++iter2_z, ++iter2_y)
// 		{
// 			ROS_INFO("X: %10f Y: %10f Z: %10f",*iter2_x,*iter2_y,*iter2_z);
// 		}
	}
	rate_.sleep();
}

bool Scanner26xxNode::startScanning()
{
	return laser_.startScanning();
}

bool Scanner26xxNode::stopScanning()
{
	return laser_.stopScanning();
}

bool Scanner26xxNode::reconnect()
{
	laser_.reconnect();
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_26xx_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
// 	SickLMS1xxNode node;
// 	
// 	if (!node.initalize()) {
// 		return 1;
// 	}
// 	
// 	node.startScanner();
	
	Scanner26xxNode scanner;
	bool scanning = scanner.startScanning();
	while(!scanning)
	{
		ROS_ERROR("Couldn't start scanning. Reconnecting!");
		scanner.reconnect();
		scanning = scanner.startScanning();
	}
	ROS_INFO("Started scanning.");
	
// 	int a = 0;
	while(!g_request_shutdown)
	{
		scanner.publish();
		
// 		if(scanner.hasNewData())
// 		{
// 			
// 			ScanProfileConvertedPtr profile = scanner.getData();
// 			if(a%200 == 0){
// 				ROS_INFO("%d",profile->profile_counter);
// 				for(int i = 0; i < profile->x.size(); ++i)
// 				{
// 					ROS_INFO("X: %10f Z: %10f",profile->x[i],profile->z[i]);
// 				}
// 			}
// 			a++;
// 		}
 		ros::spinOnce();
		
	}
	scanner.stopScanning();
	//scanner.setLaserPower(false);
	//sleep(1);
// 	node.stopScanner();
	
	ros::shutdown();
	return 0;
}
