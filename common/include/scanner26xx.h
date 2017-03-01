#ifndef _SCANNER26XX_H_
#define _SCANNER26XX_H_

#include "libllt.h"
#include <vector>
#include <queue>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>


const int scanner_resolution = 640;


class TimeSync 
{
public:
	virtual void sync_time(unsigned int profile_counter,double shutter_open,double shutter_close) = 0;
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
	
	//Each variable defines the part of the border that gets cut off. Min 0.0, Max 1.0
	MeasurementField(double left,double right,double far,double near)
	{
		assert(left >= 0.0); assert(right >= 0.0); assert(far >= 0.0); assert(near >= 0.0); 
		assert(left + right <= 1.0); assert(far + near <= 1.0); 
		ushort umax = 65535;
		z_start = near * umax;
		z_size = umax - near * umax - far * umax;
		x_start = left * umax;
		x_size = umax - left * umax - right * umax;
	}
	MeasurementField(ushort x_start, ushort x_size, ushort z_start, ushort z_size)
	: x_start(x_start),x_size(x_size),z_start(z_start),z_size(z_size)
	{
	}
		
};


struct ScanProfile
{
	guint16 z[scanner_resolution];
	guint16 x[scanner_resolution];
	guint16 padding[scanner_resolution-8];
	unsigned char timestamp[16];
};

struct ScanProfileConverted
{
	std::vector<double> x;
	std::vector<double> z;
	unsigned int profile_counter;
	double shutter_open;
	double shutter_close;
};
typedef boost::shared_ptr<ScanProfileConverted> ScanProfileConvertedPtr;

class Scanner26xx
{
private:
	bool scanning_;
	bool connected_;
	bool need_time_sync_;
	
	unsigned int idle_time_;
	unsigned int shutter_time_;
	unsigned int container_size_;
	boost::mutex mutex_;
	unsigned int fieldCount_;
	TimeSync* time_sync_;
	Notifyee*  notifyee_;
	MeasurementField field_;
	std::string serial_number_;
	
	LLT llt_;
	
	std::vector<ScanProfile> container_buffer_;
	std::queue<ScanProfileConvertedPtr> profile_queue_;
	bool connect();
	bool disconnect();
	bool initialise();
	
	void control_lost_callback(ArvGvDevice *gv_device);	
	void new_profile_callback (const void * data, size_t data_size);
	static void control_lost_callback_wrapper(ArvGvDevice *gv_device, gpointer user_data);	
	static void new_profile_callback_wrapper (const void * data, size_t data_size, gpointer user_data);
	void setMeasuringField(ushort x_start, ushort x_size, ushort z_start, ushort z_size);
	void WriteCommand(unsigned int command, unsigned int data);
	void WriteValue2Register(unsigned short value);
	
public:
	
	Scanner26xx(TimeSync* time_sync,Notifyee* notifyee,unsigned int shutter_time, unsigned int idle_time, unsigned int container_size,MeasurementField field,std::string serial_number);
	~Scanner26xx();
	
	bool reconnect();
	
	bool startScanning();
	bool stopScanning();
	bool setLaserPower(bool on);
	bool hasNewData();
	ScanProfileConvertedPtr getData();
	

};

#endif