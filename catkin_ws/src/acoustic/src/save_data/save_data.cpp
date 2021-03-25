/*program for saving the ROS msg hydrophoneData as wave file in sd card
  File_path : as config file
  File_length : as config file
  Num_channels : 2
  Resolutions : 32
  written by Shane 
  2020_12_14
 */
#include<iostream>
#include<string>
#include<cstdio>
#include<vector>
#include<ctime>
#include<cmath>
//for ROS
#include<ros/ros.h>
#include<ros/console.h>
#include"ntu_msgs/HydrophoneData.h"


using namespace std;

// define standard wave file struct  
struct header{
    char chunk_id[4]={'R', 'I', 'F', 'F'};
    int chunk_size;
    char format[4]={'W', 'A', 'V', 'E'};
    char subchunk1_id[4] = {'f', 'm', 't', ' '};
    int subchunk1_size = 16;
    short int audio_format = 1;
    short int num_channels;
    int sample_rate;
    int byte_rate;
    short int block_align;
    short int bits_per_sample;
    char subchunk2_id[4] = {'d', 'a', 't', 'a'};
    int subchunk2_size;    
}header_file;

// Define a function for setting wav headerfile 
void setHeaderFile(unsigned int sample_count, int fs){
    short int num_channels = 2; // based on your msg type
    int resolution = 32;        // 32 bits

    header_file.chunk_size = sample_count*num_channels*resolution/8+44;
    header_file.num_channels = (short int)num_channels;
    header_file.sample_rate = fs;
    header_file.byte_rate = fs*resolution/8*num_channels;
    header_file.block_align = (short int)(resolution/8*num_channels);
    header_file.bits_per_sample = (short int)(resolution);
    header_file.subchunk2_size = sample_count*num_channels*resolution/8;
}

// Define a funtion getting the current UTC time for the wave file name 
string getTime(){
    time_t now = time(0);
    tm *ts = localtime(&now);
    stringstream ss;
    string date;
    ss<<1900+ts->tm_year<<"-"<<1+ts->tm_mon<<\
    "-"<<ts->tm_mday<<"_"<<ts->tm_hour-8<<":"<<\
    ts->tm_min<<":"<<ts->tm_sec;
    ss>>date;
    return date;
}

class save_data_node
{
public:
    save_data_node();
    ~save_data_node();

    //ROS parameter
    ros::NodeHandle nh_private;
    ros::NodeHandle nh_public;
    ros::Subscriber sub;

private:
    void push(const ntu_msgs::HydrophoneData &);
    
    // config file parameters
    string  FILE_PATH_;
    int FILE_LENGTH_;

    // member variable
    unsigned int m_count;   // count the data length
    int m_fs;               // sampling rate
    FILE* m_fp;               // file pointer 
};

/*                                   */
/************ Constructor ************/
/*                                   */
save_data_node::save_data_node():
nh_private("~"), m_count(0)
{
    // Get setting from yaml file
    nh_private.getParam("FILE_PATH_", FILE_PATH_);
    nh_private.getParam("FILE_LENGTH_", FILE_LENGTH_);

    ROS_INFO("setting success\n");
    ROS_INFO("file path:\t\t%s", FILE_PATH_.c_str());
    ROS_INFO("file length(min):\t%d", FILE_LENGTH_);

    // Set ROS subscriber
    sub = nh_public.subscribe("/get_sound_data_for2i2/hydrophone_data", 1000, &save_data_node::push, this);
}

/*                                   */
/************* Destructor ************/
/*                                   */
save_data_node::~save_data_node(){
    fseek(m_fp, 0, SEEK_SET);
    setHeaderFile(m_count, m_fs);
    fwrite(&header_file, 44, 1, m_fp);
    fclose(m_fp);
    ROS_INFO("CLOSING FILE!!!");
}

// Define the push callback function 
void save_data_node::push(const ntu_msgs::HydrophoneData &msg){
    clock_t begin = clock();
    if(m_count==0){
        string filename = getTime()+".wav";
        ROS_INFO_STREAM("OPENNING NEW FILE: "<<filename<<" !!!");
        filename = FILE_PATH_ + filename;
        m_fp = fopen(filename.c_str(), "wb");
        fseek(m_fp, 44, SEEK_SET);
    }
    vector<double> ch1 = msg.data_ch1;
    vector<double> ch2 = msg.data_ch2;
    int length = msg.length;
    m_fs = msg.fs;
    unsigned int MAX = FILE_LENGTH_*60*m_fs;
    int data;
    for(int i=0;i<length;i++){
        data = (int)(ch1.at(i)*pow(2, 31));
        fwrite(&data, 4, 1, m_fp);
        data = (int)(ch2.at(i)*pow(2, 31));
        fwrite(&data, 4, 1, m_fp);
    }
    m_count += length;
    if(m_count>=MAX){
        fseek(m_fp, 0, SEEK_SET);
        setHeaderFile(m_count, m_fs);
        fwrite(&header_file, 44, 1, m_fp);
        m_count = 0;
        fclose(m_fp);
        ROS_INFO("CLOSING FILE !!!");
    }
    clock_t end = clock();
    double elapsed_secs = double(end-begin)/CLOCKS_PER_SEC;
    ROS_INFO_STREAM("Saving "<<(double)length/m_fs<<" sec data takes "<<elapsed_secs<<" sec");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "save_data_node");
    save_data_node save_obj;
    ros::spin();
    return 0;
}