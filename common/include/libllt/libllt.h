#ifndef LLT_H
#define LLT_H

#ifdef __cplusplus
extern "C" {
#endif

    #include <libmescan.h>

#ifdef __cplusplus
}
#endif

#include <LLTDataTypes.h>

/* Non-member functions */
int SetPathtoDeviceProperties (const char * path_to_dev_prop);

/* LLT Class */
class LLT
{
    public:
        explicit LLT();
        ~LLT();

        /* Init and connect */
        int Connect (void);
        int Disconnect (void);
        int SetDeviceInterface(const char * dev_interface);

        /* Get info, features and current parameter settings */
        int GetLLTType (TScannerType * scanner_type);
        int GetDeviceName (const char ** dev_name, const char **  vendor_name);
        int GetLLTVersion (const char ** version);
        int GetProfileConfig (TProfileConfig * profile_config);
        int GetFeature (guint32 feature_register, guint32 * value);
        int GetPartialProfile(TPartialProfile * partial_profile);
        int GetProfileContainerSize(guint32 * width, guint32 * height);
        int GetMaxProfileContainerSize(guint32 * max_width, guint32 * max_height);
        int GetBufferCount (guint32 * buffer_count);
        int GetResolutions (guint32 * resolutions, guint32 resolutions_size);
        int GetResolution(guint32 * resolution);
        int GetPartialProfileUnitSize (guint32 * unit_size_point, guint32 * unit_size_data);
        int GetMinMaxPacketSize (guint32 * min_packet_size, guint32 * max_packet_size);
        int GetPacketSize (guint32 * packet_size);

        /* Set features and parameters */
        int SetProfileConfig (TProfileConfig profile_config);
        int SetFeature (guint32 feature_register, guint32 value);
        int SetResolution(guint32 resolution);
        int SetProfileContainerSize(guint32 width, guint32 height);
        int SetPartialProfile (TPartialProfile partial_profile);
        int SetBufferCount (guint32 buffer_count);
        int SetPacketSize (guint32 packet_size);

        /* Handle User Modes */
        int GetActualUserMode(guint32 * current_user_mode, guint32 * available_usermodes);
        int ReadWriteUserModes(gboolean read_write, guint32 user_mode);

        /* Calibration */
        int SetCustomCalibration(double center_x, double center_z, double angle, double shift_x, double shift_z);
        int ResetCustomCalibration();

        /* Callback handling */
        int RegisterBufferCallback(gpointer callback_func, gpointer user_data);
        int RegisterControlLostCallback(gpointer control_lost_func, gpointer user_data);
        void BufferCallback (ArvStream * stream);
        void ControlLostCallback (ArvGvDevice * device);

        /* Transmission */
        int TransferProfiles (TTransferProfileType transfer_type, bool start_stop);

        ArvCamera * camera;      
        ApplicationData appData;
        ArvStream * stream;
        ArvDevice * device;        
        TProfileConfig masterProfileConfig;
        TPartialProfile masterPartialProfile;
        TTransferProfileType masterProfileType;
        guint32 masterResolution;
        guint32 bufferCt;
        MEDeviceData deviceData;

    private:
		pthread_mutex_t mutex;
		pthread_cond_t cond;
        gpointer userDataCb;
        gpointer userDataCl;
        const char * devId;
        gboolean isTransmitting, isConnected;     
        void (*vRegisterBufferCallback)(const void * , size_t, gpointer);
        void (*vRegisterControlLostCallback)(ArvGvDevice *, gpointer);
};

#endif // LLT_H
