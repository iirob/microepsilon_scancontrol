/*****
	libmescan
	version: 1.0
	Author:  Daniel Rauch

	dependencies: libavaris-0.4
*****/

#ifndef MECONNECT_H
#define MECONNECT_H

#define __STDC_FORMAT_MACROS

#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <arv.h>

#ifdef __cplusplus
extern "C" {
#endif

/***
   structs
***/

typedef struct {
	GMainLoop * main_loop;
	double scaling;
	double offset;
	int image_width;
	int image_height;
} ApplicationData;

typedef struct {
	int32_t device_series;
	double scaling;
	double offset; 
	int32_t max_packet_size; 
	int32_t max_frequency; 
	int32_t	post_proc; 
	double	min_x_display; 
	double	max_x_display; 
	double	min_y_display; 
	double	max_y_display; 
	int32_t rotate_image; 
	double min_width;		
} MEDeviceData;

typedef struct TPartialProfile
{
	unsigned int nStartPoint;
	unsigned int nStartPointData;
	unsigned int nPointCount;
	unsigned int nPointDataWidth;
} TPartialProfile;

typedef struct EHANDLE {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    bool triggered;
}EHANDLE;

/***
   extern functions
***/

int ConvertProfile2Values (const unsigned char * profile_data, unsigned int profile_data_size, const ApplicationData * app_data, unsigned int resolution, unsigned int reflection, unsigned short * width, unsigned short * intens, unsigned short * threshold, double * x, double * z, unsigned int * m0, unsigned int * m1);
int ConvertPartProfile2Values (const unsigned char * profile_data, unsigned int profile_data_size, const ApplicationData * app_data, TPartialProfile * partial_profile, unsigned int reflection, unsigned short * width, unsigned short * intens, unsigned short * threshold, double * x, double * z, unsigned int * m0, unsigned int * m1);
int InitDevice (const char * camera_id_name, MEDeviceData * dev_data, const char * path_to_dev_prop);
int GetDeviceInterfaces(char * interfaces[], unsigned int interfaces_size);
int Timestamp2TimeAndCount(const unsigned char * profile_data, double * shutter_open, double * shutter_close, unsigned int * profile_count);

void CreateEvent(EHANDLE* event);
void SetEvent(EHANDLE* event);
void FreeEvent(EHANDLE *ev);
void ResetEvent(EHANDLE* event);
int WaitForSingleObject(EHANDLE* event, int timeout);

#ifdef __cplusplus
}
#endif

#endif // MECONNECT_H
