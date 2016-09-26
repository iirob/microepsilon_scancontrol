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
	
public:
	
	Scanner26xx(TimeSync* time_sync);
	~Scanner26xx();
	
	bool reconnect();
	
	bool startScanning();
	bool stopScanning();
	bool setLaserPower(bool on);
	bool hasNewData();
	ScanProfileConvertedPtr getData();
	

};

#endif