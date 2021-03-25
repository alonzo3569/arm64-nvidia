/* For RobotX competition
   Recorder controll part is written by CT-Hung and modified to ROS version by Sam Liu
*/

#include <alsa/asoundlib.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <deque>
#include <vector>
#include <string>
#include <signal.h>

// For ROS
#include <ros/ros.h>
#include "ntu_msgs/HydrophoneData.h"

using namespace std;

class hydrophone_data_node
{
    public:
        hydrophone_data_node();                     // Constructor
        ~hydrophone_data_node();                    // Destructor
        void run(void);

        // ROS parameter
        ros::NodeHandle nh;                         // Private node handler
        ros::Publisher pub_sound;                   // Private publisher instance
        ntu_msgs::HydrophoneData hydro_msg;      // Hydrophone data message

    private:
        bool setRecorderParams(void);
        void Capture(void);

        // Recorder parameter
        string DEVICE_NAME_;
        string pcm_name_;
        snd_pcm_t *pcm_handle_;
        snd_pcm_hw_params_t *pcm_params_;
        snd_pcm_uframes_t pcm_frames_;
        unsigned int pcm_sampleRate_;
        unsigned int pcm_available_channels_;
        unsigned int pcm_using_channels_;
        unsigned int pcm_loops_;
        int pcm_dir_;
        int pcm_period_size_;
        int pcm_bits_;
        int pcm_send_size_;
        char *pcm_period_buffer_;
        double pcm_recordTime_;
};

/*                                    */
/************ Constructor *************/
/*                                    */
hydrophone_data_node::hydrophone_data_node()
{
    // Setup the publisher
    pub_sound = nh.advertise<ntu_msgs::HydrophoneData>("hydrophone_data", 10);

    double tmp;
    // Import parameter from yaml file
    if (!ros::param::get("~DEVICE_NAME_", DEVICE_NAME_))
        DEVICE_NAME_ = string("Focusrite 18i8 2nd");
    if (!ros::param::get("~pcm_name_", pcm_name_))
        pcm_name_ = string("hw:0,0");
    if (!ros::param::get("~pcm_frames_", tmp))
        pcm_frames_ = 100;
    else pcm_frames_ = tmp;
    if (!ros::param::get("~pcm_sampleRate_", tmp))
        pcm_sampleRate_ = 96000;
    else  pcm_sampleRate_ = tmp;
    if (!ros::param::get("~pcm_available_channels_", tmp))
        pcm_available_channels_ = 10;
    else pcm_available_channels_ = tmp;
    if (!ros::param::get("~pcm_using_channels_", tmp))
        pcm_using_channels_ = 10;
    else pcm_using_channels_ = tmp;
    if (!ros::param::get("~pcm_bits_", pcm_bits_))
        pcm_bits_ = sizeof(int32_t) * 8;
    if (!ros::param::get("~pcm_recordTime_", tmp))
        pcm_recordTime_ = 1;
    else pcm_recordTime_ = tmp;
    

    // Initialize message data
    hydro_msg.data_type = "int32_t";
    hydro_msg.data_ch1.clear();
    hydro_msg.data_ch2.clear();
    hydro_msg.data_ch3.clear();
    hydro_msg.data_ch4.clear();
}


/*                                    */
/************* Destructor *************/
/*                                    */
hydrophone_data_node::~hydrophone_data_node()
{
    // Stop the sound card object
    snd_pcm_drain(pcm_handle_);
    snd_pcm_close(pcm_handle_);
    free(pcm_period_buffer_);
    cout << DEVICE_NAME_ << " stop recording." << endl;
}

bool hydrophone_data_node::setRecorderParams(void)
{
    int rc;
    /* Open PCM device for recording. */
    rc = snd_pcm_open(&pcm_handle_, pcm_name_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0){
        cout << "Can not open record device: " << pcm_name_ << endl;
    }else
        cout << "Open device sucessfully: " << pcm_name_ << endl;
  
    /* Allcoate a hardware params object. */ 
    snd_pcm_hw_params_alloca(&pcm_params_);

    /* Fill it in with default values. */ 
    snd_pcm_hw_params_any(pcm_handle_, pcm_params_);

    /* Set the params to device. */
    /*Interleaved mode. */
    snd_pcm_hw_params_set_access(pcm_handle_, pcm_params_, SND_PCM_ACCESS_RW_INTERLEAVED);
  
    /* Signed 32-bit little-endian format. */
    snd_pcm_hw_params_set_format(pcm_handle_, pcm_params_, SND_PCM_FORMAT_S32_LE);

    /* Set channel. */
    snd_pcm_hw_params_set_channels(pcm_handle_, pcm_params_, pcm_available_channels_);

    /* Set sample rate. */
    snd_pcm_hw_params_set_rate_near(pcm_handle_, pcm_params_, &pcm_sampleRate_, &pcm_dir_);
  
    /* Set period size to frames. */
    snd_pcm_hw_params_set_period_size_near(pcm_handle_, pcm_params_, &pcm_frames_, &pcm_dir_);

    /* Write the params to the driver. */
    rc = snd_pcm_hw_params(pcm_handle_, pcm_params_);
    if(rc < 0){
        cout << "Unable to set hw params. " << endl;
        return false;
    }
    else
        cout << "Set params sucessfully. " << endl;
    
    /* Decide the period size and buffer. */
    snd_pcm_hw_params_get_period_size(pcm_params_, &pcm_frames_, &pcm_dir_);
    pcm_period_size_ = pcm_frames_ * pcm_bits_ * pcm_available_channels_ / 8; //16bits // units is byte.
    pcm_period_buffer_ = (char *) malloc(pcm_period_size_);
    return true;
}

void hydrophone_data_node::Capture(void)
{
    int rc;

    rc = snd_pcm_readi(pcm_handle_, pcm_period_buffer_, pcm_frames_);
    if(rc == -EPIPE){
        cout << "Overrun occurred" << endl;
        snd_pcm_prepare(pcm_handle_);
    }else if(rc < 0){
        string error = snd_strerror(rc);
        cout << "Error from read: " << error << endl;
    }else if(rc != (int)pcm_frames_){
        cout << "Short read, read wrong frames: " << rc << endl;
    }

    for(int i = 0; i < pcm_period_size_ - pcm_available_channels_ * 4 + 1; i = i + pcm_available_channels_ * 4){
        //int sum1 = (pcm_period_buffer_[i]<<0) | (pcm_period_buffer_[i+1]<<8) | (pcm_period_buffer_[i+2]<<16) | (pcm_period_buffer_[i+3]<<24);

        int sum1 = (unsigned char)pcm_period_buffer_[i]+256*(unsigned char)pcm_period_buffer_[i+1]+256*256*(unsigned char)pcm_period_buffer_[i+2]+256*256*256*pcm_period_buffer_[i+3];
        int sum2 = (unsigned char)pcm_period_buffer_[i+4]+256*(unsigned char)pcm_period_buffer_[i+5]+256*256*(unsigned char)pcm_period_buffer_[i+6]+256*256*256*pcm_period_buffer_[i+7];
        int sum3 = (unsigned char)pcm_period_buffer_[i+8]+256*(unsigned char)pcm_period_buffer_[i+9]+256*256*(unsigned char)pcm_period_buffer_[i+10]+256*256*256*pcm_period_buffer_[i+11];
        int sum4 = (unsigned char)pcm_period_buffer_[i+12]+256*(unsigned char)pcm_period_buffer_[i+13]+256*256*(unsigned char)pcm_period_buffer_[i+14]+256*256*256*pcm_period_buffer_[i+15];
        hydro_msg.data_ch1.push_back(sum1); //channel 1 data
        hydro_msg.data_ch2.push_back(sum2); //channel 2 data 
        hydro_msg.data_ch3.push_back(sum3); //channel 3 data
        hydro_msg.data_ch4.push_back(sum4); //channel 4 data
    }
}

void hydrophone_data_node::run(void)
{
    // Set the parameter of recorder and check if the command succeeded
    bool enable_recorder = setRecorderParams();
    if(enable_recorder == false)
    {
        ROS_ERROR("ROS shutdown");
        ros::shutdown();
    }

    hydro_msg.fs = pcm_sampleRate_;
    hydro_msg.bits = pcm_bits_;
    // Loop
    while (ros::ok())
    {
        Capture();
        if(hydro_msg.data_ch1.size() >= ((int)pcm_sampleRate_ * pcm_recordTime_)) // send data every pcm_recordtime_ seconds
        {
            hydro_msg.length = hydro_msg.data_ch1.size();
           ROS_INFO("Published %d samples data.", (int)hydro_msg.length);

            pub_sound.publish(hydro_msg);

            hydro_msg.data_ch1.clear();
            hydro_msg.data_ch2.clear();
            hydro_msg.data_ch3.clear();
            hydro_msg.data_ch4.clear();
        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "hydrophone_data_node");
    
    // Create hydrophone object
    hydrophone_data_node hydro_obj;

    // Run
    hydro_obj.run();

    return 0;
}
