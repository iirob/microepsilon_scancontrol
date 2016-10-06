#include "ros/ros.h"
#include "scanner26xx.h"
#include "../../../cob_driver/cob_sick_lms1xx/common/include/lms1xx.h"
// #include <signal.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// sig_atomic_t volatile g_request_shutdown = 0;
// void mySigIntHandler(int sig)
// {
// 	g_request_shutdown = 1;
// }

double average(double a, double b)
{
	return (a+b)/2.0;
}

class Scanner26xxNode : public TimeSync, Notifyee
{
public:
	
	Scanner26xxNode(unsigned int shutter_time, unsigned int idle_time, unsigned int container_size,double lag_compensation);
	
	void publish();
	bool startScanning();
	bool stopScanning();
	bool reconnect();
	virtual void sync_time(unsigned int profile_counter,double shutter_open,double shutter_close);
	virtual void notify();
	
private:
	
	void initialiseMessage();
	

	ros::Publisher scan_pub_;
	ros::NodeHandle nh_;
	// laser data
	Scanner26xx laser_;
	int last_second_;
	double lag_compensation_;
	// published data
	sensor_msgs::PointCloud2 cloud_msg_;
	// parameters
	ros::Duration shutter_close_sync_;
};

Scanner26xxNode::Scanner26xxNode(unsigned int shutter_time, unsigned int idle_time, unsigned int container_size,double lag_compensation) 
								: laser_(this,this,shutter_time,idle_time,container_size), lag_compensation_(lag_compensation)
{
	scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_scan",500);
	initialiseMessage();
	ROS_INFO("Connecting to Laser");
}

void Scanner26xxNode::sync_time(unsigned int profile_counter, double shutter_open, double shutter_close)
{
	ROS_DEBUG("New Timestamp: %d %9f",profile_counter,average(shutter_open,shutter_close));
	shutter_close_sync_ = ros::Time::now() - ros::Time(average(shutter_open,shutter_close)) - ros::Duration(lag_compensation_);
	last_second_ = 0;
}

void Scanner26xxNode::notify()
{
	publish();
}



void Scanner26xxNode::initialiseMessage()
{
	cloud_msg_.header.frame_id = "arm_scanner_link";
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
	
	while(laser_.hasNewData())
	{
		sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
		sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
		ScanProfileConvertedPtr data = laser_.getData();
		ros::Time profile_time(average(data->shutter_open,data->shutter_close));
		if(profile_time.toSec() - last_second_ < 0)
		{
			shutter_close_sync_ += ros::Duration(128);
		}
		last_second_ = profile_time.toSec();
		cloud_msg_.header.stamp = profile_time + shutter_close_sync_;
		++cloud_msg_.header.seq;
		ROS_DEBUG_STREAM(profile_time << " " << cloud_msg_.header.stamp);
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
	ros::init(argc, argv, "scan_26xx_node");
// 	ros::init(argc, argv, "scan_26xx_node", ros::init_options::NoSigintHandler);
// 	signal(SIGINT, mySigIntHandler);

	
	ros::NodeHandle nh_private("~");
	
	int shutter_time;
	int idle_time;
	int container_size;
	double lag_compensation;
	if(!nh_private.getParam("shutter_time",shutter_time))
	{
		ROS_ERROR("You have to specify parameter shutter_time!");
		return -1;
	}
	if(!nh_private.getParam("idle_time",idle_time))
	{
		ROS_ERROR("You have to specify parameter idle_time!");
		return -1;
	}
	if(!nh_private.getParam("container_size",container_size))
	{
		ROS_ERROR("You have to specify parameter container_size!");
		return -1;
	}
	if(!nh_private.getParam("lag_compensation",lag_compensation))
	{
		lag_compensation = 0.0;
	}
	ROS_INFO("Shutter Time: %dms Idle Time: %dms Frequency: %.2fHz",shutter_time/100,idle_time/100,100000.0/(shutter_time+idle_time));
	ROS_INFO("Profiles for each Container: %d",container_size);
	ROS_INFO("Lag compensation: %.3fms",lag_compensation*1000);
	
	
	Scanner26xxNode scanner(shutter_time,idle_time,container_size,lag_compensation);
	bool scanning = scanner.startScanning();
	while(!scanning)
	{
		ROS_ERROR("Couldn't start scanning. Reconnecting!");
		scanner.reconnect();
		scanning = scanner.startScanning();
	}
	ROS_INFO("Started scanning.");
	
// 	int a = 0;
// 	ros::Rate rate(1000);
// 	tf::TransformBroadcaster tf_bc;
// 	tf::TransformListener tf_li;
// 	while(!g_request_shutdown)
// 	{
		
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
// 		static tf::StampedTransform st_transform_last_loop;
// 		static tf::StampedTransform st_transform_T_1;
// 		tf::StampedTransform st_transform;
// 		try
// 		{
// 			tf_li.lookupTransform("world","force_tool_base_link",ros::Time(0),st_transform);
// 			if(st_transform == st_transform_last_loop)
// 			{
// 				ros::Duration delta_t = st_transform.stamp_ - st_transform_T_1.stamp_;
// 				ros::Duration delta_t_now = ros::Time::now() - st_transform.stamp_;
// 				double time_ratio = delta_t_now.toSec() / delta_t.toSec();
// 				
// 				tf::Vector3 new_vec3 = st_transform_T_1.getOrigin().lerp(st_transform.getOrigin(),1.0+time_ratio);
// 				tf::Quaternion new_quat = st_transform_T_1.getRotation().slerp(st_transform.getRotation(),1.0+time_ratio);
// 				tf::Transform transform;
// 				transform.setOrigin(new_vec3);
// 				transform.setRotation(new_quat);
// 				tf_bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "extrapolated_force_tool_base_link"));
// 			}
// 			else
// 			{		
// 				st_transform_T_1 = st_transform_last_loop;
// 			}
// 			st_transform_last_loop = st_transform;
// 		}
// 		catch(const tf2::TransformException& e)
// 		{
// 			;
// 		}
//  		ros::spinOnce();
// 		rate.sleep();
// 		
// 	}
	ros::spin();
	scanner.stopScanning();
	
// 	ros::shutdown();
	return 0;
}
