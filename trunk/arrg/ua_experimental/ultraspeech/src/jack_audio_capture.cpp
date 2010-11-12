#include <ros/ros.h>
#include <iostream>
#include <jack/jack.h>
#include <sys/types.h>
#include <string>

#include "ultraspeech/AudioStream.h"
#include "ultraspeech/Control.h"

const static int SAMPLE_RATE = 96000; // this is dictated by the Saffire
const static int NUM_CHANNELS = 1; // call this node for each channel

jack_port_t *input_port_;
jack_client_t *client_;
std::string subdir_;
ros::Subscriber control_sub_;
ros::Publisher pub_;
uint32_t which_channel_;
int run_status_;

int process_cb(jack_nframes_t nframes, void *arg)
{
  using std::cout;
  using std::endl;
  
  static ultraspeech::AudioStream audio_msg;
  
  jack_default_audio_sample_t *in = (jack_default_audio_sample_t *) jack_port_get_buffer(input_port_, nframes);

  //if (run_status_ == 1) {
    audio_msg.header.stamp = ros::Time::now();
    audio_msg.num_channels = NUM_CHANNELS;
    audio_msg.sample_rate = SAMPLE_RATE;
    audio_msg.which_channel = which_channel_;

    audio_msg.samples.resize(nframes * NUM_CHANNELS);
    float *inp = (float *)in;
    for (uint32_t i = 0; i < nframes; i++)
    {
      float val;
      for(uint32_t j = 0; j < NUM_CHANNELS; ++j){
	val = *inp++;
	audio_msg.samples[i*NUM_CHANNELS + j] = val;
      }
    }
  
    pub_.publish(audio_msg);
  //}

  return 0;
}
  
void jack_shutdown_cb(void *arg)
{
  exit(1);
}

void control_cb(const ultraspeech::ControlConstPtr& msg)
{
  using std::cout;
  using std::endl;
  
  run_status_ = msg->run;
  subdir_ = (std::string)(msg->directory + "/audio/");
  std::cout << "run_status changed: " << run_status_ << std::endl;
  
}
  
void JackAudioCapture(const ros::NodeHandle& nh,
		      const char* clientname,
		      int channel)
{
  using std::cerr;
  using std::cout;
  using std::endl;
  
  const char **ports_;
  channel = channel - 1; //this tells which port to connect to
  
  std::string topic = nh.resolveName("control");
  ros::NodeHandle local_nh("~");
  pub_ = local_nh.advertise<ultraspeech::AudioStream>("/audio", 1);
    
  if ((client_ = jack_client_new(clientname)) == 0){
    std::cerr << "Error: jack server not running?" << std::endl;
    return;
  }
    
  jack_set_process_callback(client_, process_cb, 0);
  jack_on_shutdown(client_, jack_shutdown_cb, 0);
  input_port_ = jack_port_register(client_, "input", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
  
  if (jack_activate(client_)) {
    std::cerr << "Error: cannot activate client" << std::endl;
    return;
  }
  
  if ((ports_ = jack_get_ports(client_, NULL, NULL, JackPortIsPhysical|JackPortIsOutput)) == NULL) {
    std::cerr << "Error: Cannot find any physical capture ports" << std::endl;
    return;
  }
   
  if (jack_connect(client_, ports_[channel], jack_port_name(input_port_))) {
    std::cerr << "Error: Cannot connect input ports" << std::endl;
  }
  
  run_status_ = 0;

}

int main(int argc, char **argv)
{
  int channel;
  channel = atoi(argv[2]);
  which_channel_ = (uint32_t)channel;
  
  if (channel > 18) {
    std::cerr << "Channel must be 1-18 (for Focusrite Saffire)" << std::endl;
    exit(1);
  }
  if (channel < 1) {
    std::cerr << "Channel must be 1-18 (for Focusrite Saffire)" << std::endl;
    exit(1);
  }    

  ros::init(argc, argv, argv[1], ros::init_options::AnonymousName);
  ros::NodeHandle n;

  JackAudioCapture(n, argv[1], channel);

  ros::spin();

  return 0;
}