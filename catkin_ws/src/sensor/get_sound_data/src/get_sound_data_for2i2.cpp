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
        string DEVICE_NAME_;
        string pcm_name_;
        snd_pcm_t *pcm_handle_;
        snd_pcm_hw_params_t *pcm_params_;
        snd_pcm_uframes_t pcm_frames_;
        unsigned int pcm_sampleRate_;
        unsigned int pcm_available_channels_;
        unsigned int pcm_using_channels_;
        //unsigned int pcm_recordTime_;
        unsigned int pcm_loops_;
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
hydrophone_data_node::hydrophone_data_node():
//    DEVICE_NAME_("Focusrite 18i8 2nd"),    // Initial parameters
//    pcm_name_("hw:1,0"),
//    pcm_frames_(100),
//    pcm_sampleRate_(192000),
//    pcm_available_channels_(2),
//    pcm_using_channels_(2),
//    //pcm_recordTime_(10),
//    pcm_loops_(0),    
	pcm_bits_(sizeof(int32_t) * 8),
	nh("~")
{
	nh.getParam("DEVICE_NAME_", DEVICE_NAME_);
	nh.getParam("pcm_name_", pcm_name_);
	double tmp;
	nh.getParam("pcm_available_channels_", tmp);
	pcm_available_channels_ = tmp;
	nh.getParam("pcm_frames_", tmp);
    pcm_frames_ = tmp;
	nh.getParam("pcm_loops_", tmp);
	pcm_loops_ = tmp;
	nh.getParam("pcm_sampleRate_", tmp);
	pcm_sampleRate_ = tmp;
	nh.getParam("pcm_using_channels_", tmp);
    pcm_using_channels_ = tmp;
	nh.getParam("msg_length_", tmp);
	msg_length_ = tmp;
	ROS_INFO("set msg length: %f", msg_length_);

    // Setup the publisher
    pub_sound = nh.advertise<ntu_msgs::HydrophoneData>("hydrophone_data", 10);

    // Initialize message data
	hydro_msg.fs = pcm_sampleRate_;
    hydro_msg.data_ch1.clear();
    hydro_msg.data_ch2.clear();
    // hydro_msg.data_ch3.clear();
    // hydro_msg.data_ch4.clear();
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
    std::cout << 13 << std::endl;
    snd_pcm_hw_params_set_format(pcm_handle_, pcm_params_, SND_PCM_FORMAT_S16_LE);
    std::cout << 16 << std::endl;

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

    std::cout << 0 << std::endl;
    snd_pcm_hw_params_get_period_size(pcm_params_, &pcm_frames_, &pcm_dir_);
    std::cout << 1 << std::endl;
    pcm_period_size_ = pcm_frames_ * pcm_bits_ * pcm_available_channels_ / 8; //16bits // units is byte.
    std::cout << 2 << std::endl;
    pcm_period_buffer_ = (char *) malloc(pcm_period_size_);
    std::cout << 3 << std::endl;

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

    for(int i = 0; i < pcm_period_size_ - pcm_available_channels_ * 2 + 1; i = i + pcm_available_channels_ * 2){
        int sum1 = (unsigned char)pcm_period_buffer_[i]+256*(unsigned char)pcm_period_buffer_[i+1];
        int sum2 = (unsigned char)pcm_period_buffer_[i+2]+256*(unsigned char)pcm_period_buffer_[i+3];
        double data;
        data = sum1/pow(2,31);
        hydro_msg.data_ch1.push_back(data); //channel 1 data
        data = sum2/pow(2,31);
        hydro_msg.data_ch2.push_back(data); //channel 2 data 
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
