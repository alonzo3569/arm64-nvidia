/* For RobotX competition
   Recorder controll part is written by CT-Hung, Sam Liu, Shane, logan
*/

#include <alsa/asoundlib.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <deque>
#include <vector>
#include <string>
#include <signal.h>
#include <cmath>

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
        string pcm_id_;
        snd_pcm_t *pcm_handle_;
        snd_pcm_hw_params_t *pcm_params_;
        snd_pcm_uframes_t pcm_frames_;
        unsigned int pcm_sampleRate_;
        unsigned int pcm_available_channels_;
        //unsigned int pcm_using_channels_;  //useless
        //unsigned int pcm_recordTime_;
        //unsigned int pcm_loops_;       //useless
        int pcm_dir_;
        int pcm_period_size_;
        int pcm_bits_;
        int pcm_send_size_;
        char *pcm_period_buffer_;
	double msg_length_;
};


/*                                    */
/************ Constructor *************/
/*                                    */
hydrophone_data_node::hydrophone_data_node(){
    //OnstartUp
    double tmp;
    ros::param::get("~pcm_id", pcm_id_);
    cout << "pcm_id :" << pcm_id_ << endl;

    ros::param::get("~pcm_available_channels", tmp);
    pcm_available_channels_ = tmp;
    cout << "pcm_available_channels :" << pcm_available_channels_ << endl;

    ros::param::get("~pcm_frames", tmp);
    pcm_frames_ = tmp;
    cout << "pcm_frames :" << pcm_frames_ << endl;

    ros::param::get("~pcm_sampleRate", tmp);
    pcm_sampleRate_ = tmp;
    cout << "pcm_sampleRate :" << pcm_sampleRate_ << endl;

    ros::param::get("~pcm_bits", tmp);
    pcm_bits_ = tmp;
    cout << "pcm_bits :" << pcm_bits_ << endl;


    ros::param::get("~msg_length", tmp);
    msg_length_ = tmp;
    cout << "msg length :" << msg_length_ << endl;
    ROS_INFO("set msg length: %f", msg_length_);

    // Setup the publisher
    pub_sound = nh.advertise<ntu_msgs::HydrophoneData>("hydrophone_data", 10);

    // Initialize message data
    hydro_msg.fs = pcm_sampleRate_;
    hydro_msg.data_ch1.clear();
    hydro_msg.data_ch2.clear();

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
    cout << "Focusrite stop recording." << endl;
}

bool hydrophone_data_node::setRecorderParams(void) {
    int rc;
    /* Open PCM device for recording. */
    rc = snd_pcm_open(&pcm_handle_, pcm_id_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0){
        cout << "Can not open record device: " << pcm_id_ << endl;
    }else
        cout << "Open device sucessfully: " << pcm_id_ << endl;

    /* Allcoate a hardware params object. */ 
    snd_pcm_hw_params_alloca(&pcm_params_);

    /* Fill it in with default values. */ 
    snd_pcm_hw_params_any(pcm_handle_, pcm_params_);

    /* Set the params to device. */
    /*Interleaved mode. */
    snd_pcm_hw_params_set_access(pcm_handle_, pcm_params_, SND_PCM_ACCESS_RW_INTERLEAVED);

    /* Signed 32-bit little-endian format. */
    switch(pcm_bits_){
      case 16:
        snd_pcm_hw_params_set_format(pcm_handle_, pcm_params_, SND_PCM_FORMAT_S16_LE);
        break;
      case 32:
        snd_pcm_hw_params_set_format(pcm_handle_, pcm_params_, SND_PCM_FORMAT_S32_LE);
        break;
    }
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

    /* Decide the period size and buffer. */
    snd_pcm_hw_params_get_period_size(pcm_params_, &pcm_frames_, &pcm_dir_);
    pcm_period_size_ = pcm_frames_ * pcm_bits_ * pcm_available_channels_ / 8; // units is byte.
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

    switch(pcm_bits_){
      case 16:
        for(int i = 0; i < pcm_period_size_ - pcm_available_channels_ * (pcm_bits_/8) + 1; i = i + pcm_available_channels_ * (pcm_bits_/8)){
            int16_t sum1 = (unsigned char)pcm_period_buffer_[i]+256*pcm_period_buffer_[i+1];
            int16_t sum2 = (unsigned char)pcm_period_buffer_[i+2]+256*pcm_period_buffer_[i+3];
            double data;
            data = sum1/pow(2,15);
            hydro_msg.data_ch1.push_back(data); //channel 1 data
            data = sum2/pow(2,15);
            hydro_msg.data_ch2.push_back(data); //channel 2 data 
        }
     case 32:
        for(int i = 0; i < pcm_period_size_ - pcm_available_channels_ * (pcm_bits_/8) + 1; i = i + pcm_available_channels_ * (pcm_bits_/8)){
            int sum1 = (unsigned char)pcm_period_buffer_[i]+256*(unsigned char)pcm_period_buffer_[i+1]+256*256*(unsigned char)pcm_period_buffer_[i+2]+256*256*256*pcm_period_buffer_[i+3];
            int sum2 = (unsigned char)pcm_period_buffer_[i+4]+256*(unsigned char)pcm_period_buffer_[i+5]+256*256*(unsigned char)pcm_period_buffer_[i+6]+256*256*256*pcm_period_buffer_[i+7];
            double data;
            data = sum1/pow(2,31);
            hydro_msg.data_ch1.push_back(data); //channel 1 data
            data = sum2/pow(2,31);
            hydro_msg.data_ch2.push_back(data); //channel 2 data 
        }
    }
}

void hydrophone_data_node::run(void){
    // Set the parameter of recorder and check if the command succeeded
    bool enable_recorder = setRecorderParams();
    if(enable_recorder == false)
    {
        ROS_ERROR("ROS shutdown");
        ros::shutdown();
    }

    // Loop
    while (ros::ok())
    {
        Capture();
        if(hydro_msg.data_ch1.size() >= pcm_sampleRate_ * msg_length_)    // send data every 0.5 second
        {
            hydro_msg.length = hydro_msg.data_ch1.size();
            ROS_INFO("Published %d samples data.", (int)hydro_msg.data_ch1.size());

            pub_sound.publish(hydro_msg);

            hydro_msg.data_ch1.clear();
            hydro_msg.data_ch2.clear();
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
