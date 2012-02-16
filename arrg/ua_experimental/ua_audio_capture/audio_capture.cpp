///////////////////////////////////////////////////////////////////////////////
// A simple audio grabber which uses PortAudio to put an audio stream into ROS
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <signal.h>
#include <portaudio.h>
#include <boost/thread/thread.hpp>
#include "ros/ros.h"
#include "ua_audio_msgs/AudioRawStream.h"

// const static int SAMPLE_RATE = 44100; // todo: make this a parameter.
//const static int SAMPLE_RATE = 48000; // todo: make this a parameter.
unsigned int SAMPLE_RATE = 44100; // todo: make this a parameter.
unsigned int NUM_CHANNELS = 1;
ros::Publisher g_pub;
bool g_caught_sigint = false;

#define SHOW_MAX_SAMPLE
//#define GRATUITOUS_DEBUGING

static int audio_cb(const void *input, void *output,
                    unsigned long frame_count,
                    const PaStreamCallbackTimeInfo *time_info,
                    PaStreamCallbackFlags status, void *user_data)
{
  static ua_audio_msgs::AudioRawStream audio_msg;
  audio_msg.num_channels = NUM_CHANNELS;
  audio_msg.sample_rate = SAMPLE_RATE;
#ifdef GRATUITOUS_DEBUGGING
  static ros::Time prev_t;
  ros::Time t(ros::Time::now());
  ROS_DEBUG("got %lu samples, dt = %.3f\n", frame_count, (t - prev_t).to_double());
  prev_t = t;
#endif
#ifdef SHOW_MAX_SAMPLE
  float max = 0;
#endif
  audio_msg.samples.resize(frame_count * NUM_CHANNELS);
  float *inp = (float *)input;
  for (uint32_t i = 0; i < frame_count; i++)
  {
    //float val = ((float *)input)[i];
      float val;
      for(uint32_t j = 0; j < NUM_CHANNELS; ++j){
          val = *inp++;
          audio_msg.samples[i*NUM_CHANNELS + j] = val;
          //printf("%f, ",val);
#ifdef SHOW_MAX_SAMPLE
          if (val > max)
            max = val;
#endif          
      }
      //printf("\n--\n");
  }
#ifdef SHOW_MAX_SAMPLE
  printf("%.3f\r", max);
  fflush(stdout);
#endif
  //g_audio_node->publish("audio", audio_msg);
  g_pub.publish(audio_msg);
  return paContinue;
}

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_DEBUG("caught sigint, init shutdown sequence...");
}

int main(int argc, char **argv)
{
    
  if(argc > 1 && argc < 3){
      ROS_FATAL("Need exactly zero or two arguments\nUsage:\n  audio_capture  NUM_CHANNELS SAMPLE_RATE \n  audio_capture without arguments defaults to 1 channel at 441000 Hz\n");
      ROS_BREAK();
  }
  if( argc > 1){
      NUM_CHANNELS = atoi(argv[1]);        
      SAMPLE_RATE = atoi(argv[2]);
  }
  ROS_INFO("Starting with %d channels at %d Hz", NUM_CHANNELS, SAMPLE_RATE);

  PaStream *stream;
  PaError err;
  ROS_DEBUG("initializing portaudio");
  if (Pa_Initialize() != paNoError)
  {
    ROS_FATAL("unable to initialize portaudio");
    ROS_BREAK();
  }
  ROS_DEBUG("opening default audio stream");
  ros::init(argc, argv, "audio_capture", ros::init_options::NoSigintHandler);
    
  ros::NodeHandle nh("audio_capture");
  //g_audio_node = ros::NodeHandle("audio_capture");
  g_pub = nh.advertise<ua_audio_msgs::AudioRawStream>("audio", 5);
  err = Pa_OpenDefaultStream(&stream, NUM_CHANNELS, 0, paFloat32, SAMPLE_RATE, 4096,
                             audio_cb, NULL);
  if (err != paNoError)
  {
    ROS_FATAL("unable to open audio stream\n");
    ROS_BREAK();
  }
  ROS_INFO("starting audio stream, ctrl-c to stop.");
#ifdef SHOW_MAX_SAMPLE
  ROS_INFO("the printout stream shows the maximum audio sample for each portaudio snippet; use this to sanity-check your audio configuration and to set the gains inside alsamixer and on your audio hardware. You want the peaks to be near 1.0 but not quite 1.0 -- digital audio doesn't saturate very nicely.");
#endif
  if (Pa_StartStream(stream) != paNoError)
  {
    ROS_FATAL("unable to start audio stream");
    ROS_BREAK();
  }
  signal(SIGINT, sig_handler);
  boost::thread t = boost::thread::thread(boost::bind(&ros::spin));
  while (!g_caught_sigint)
    usleep(500*1000);
  ROS_DEBUG("stopping audio stream");
  t.interrupt();
  t.join();
  if (Pa_StopStream(stream) != paNoError)
  {
    ROS_FATAL("unable to stop audio stream");
    ROS_BREAK();
  }
  ROS_DEBUG("shutting down portaudio");
  if (Pa_Terminate() != paNoError)
  {
    ROS_FATAL("unable to close portaudio");
    ROS_BREAK();
  }
  
  return 0;
}

