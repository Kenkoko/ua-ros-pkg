///////////////////////////////////////////////////////////////////////////////
// This program watches a ROS audio stream and writes its clips into files
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
#include <algorithm>
#include <string>
#include <cstring>
#include <deque>
#include <vector>
#include <ros/ros.h>
#include "ultraspeech/AudioStream.h"
#include "ultraspeech/Control.h"
#include "ultraspeech/CurrentStim.h"
#include "ultraspeech/SaveFile.h"
#include <sndfile.h>
#include <iostream>

using std::deque;
using std::vector;
using std::cout;
using std::endl;

static const float CLIP_START_POWER = 0.05f;
static const float CLIP_MAINTAIN_POWER = 0.03f;
static const float WINDOW_LENGTH = 1.0; // seconds
static const float MIN_CLIP_LENGTH = 2.0; // seconds

char *convertStringToChar(const std::string &str)
{
  char *retPtr = new char[str.length() + 1];
  std::strcpy(retPtr, str.c_str());
  return retPtr;
}

class AudioWriter
{
public:
  AudioWriter(ros::NodeHandle nh, char* filename)
  : clip_num(1), clip_start(0), clip_end(0), audio_clock(0),
    clip_state(IDLE), window_power(0), run_status_(0), cut_file_(0), filename_chunk_(filename)
  {
    std::string topic = nh.resolveName("audio");
    std::string saveto = nh.resolveName("save_name");
    sub = nh.subscribe<ultraspeech::AudioStream>(topic, 5, boost::bind(&AudioWriter::audio_cb, this, _1));
    control_sub_ = nh.subscribe("/control", 1, &AudioWriter::control_cb, this);
    current_sub_ = nh.subscribe("/current_stimulus", 1, &AudioWriter::current_cb, this);
    savefile_pub_ = nh.advertise<ultraspeech::SaveFile>(saveto, 1);
      
    cout << "Existing" << endl;
  }
  
  void control_cb(const ultraspeech::ControlConstPtr& msg)
  {
    using std::cout;
    using std::endl;
    
    run_status_ = msg->run;
    subdir_ = (std::string)(msg->directory + "/" + filename_chunk_ + "/");
    cout << subdir_ << endl;
    if (run_status_ == 0)
      cut_file_ = 1; //get the last clip
    std::cout << "run_status changed: " << run_status_ << std::endl;
  }
  
  void current_cb(const ultraspeech::CurrentStimConstPtr& msg)
  {
    current_stim_ind_ = (int)msg->header.seq;
    //cut_file_ = 1;
    
  }
  
  void audio_cb(ultraspeech::AudioStreamConstPtr audio_msg)
  {
    uint32_t numChannels = audio_msg->num_channels;
    uint32_t numSamples = audio_msg->samples.size() / numChannels;
    ultraspeech::SaveFile sv_msg;
    char num[10];

    for (size_t i = 0; i < numSamples; i++)
    {
      audio_clock++;
      if (window.size() >= numChannels * WINDOW_LENGTH * audio_msg->sample_rate){
          for(uint32_t j=0; j < numChannels; ++j){        
              window.pop_front();
          }
      }
      for(uint32_t j=0; j < numChannels; ++j){        
            window.push_back(audio_msg->samples[i*numChannels+j]);
      }
      if (audio_clock % 100 == 0)
      {
        // compute RMS power
        float sum_squares = 0;
        int win_idx = 0;
        for (deque<float>::iterator j = window.begin(); j != window.end();
             ++j, win_idx++)
          sum_squares += (*j) * (*j);// * ((float)win_idx / window.size());
        window_power = sqrt(sum_squares / window.size() );
      }

      switch (clip_state)
      {
        case IDLE:
	  if (run_status_ == 1)
          {
            printf("clip start\n");
            clip_start = audio_clock;
	    sv_msg.header.stamp = audio_msg->header.stamp;
	    std::sprintf(num, "%.06d", clip_num);
	    std::string filename(subdir_ + filename_chunk_ + "_" + num +".wav");
	    sv_msg.filepath = filename;
	    savefile_pub_.publish(sv_msg);
	    
            clip_state = CLIP_START;
            clip.clear();
            clip.reserve(window.size() * numChannels);
            for (deque<float>::iterator j = window.begin();
                 j != window.end(); ++j)
              clip.push_back(*j);
          }
          break;
        case CLIP_START:
          for(uint32_t j=0; j < numChannels; ++j){        
            clip.push_back(audio_msg->samples[i* numChannels]);
          }
          if (audio_clock - clip_start < audio_msg->sample_rate * WINDOW_LENGTH )
          {
            clip_state = IN_CLIP;
          }
          else
          {
            clip_state = IDLE;
            printf("audio power never ramped up; it's not a real clip.\n");
          }
          break;
        case IN_CLIP:
          for(uint32_t j=0; j < numChannels; ++j){        
            clip.push_back(audio_msg->samples[i* numChannels]);
          }
          if (cut_file_ == 1)
          {
	    cut_file_ = 0;
            clip_state = IDLE;
            const float clip_len = (float)clip.size() / audio_msg->sample_rate / numChannels;
            printf("good clip, length = %.2f seconds\n", clip_len);
            // normalize it
            float max_amp = 0, mean = 0;
            for (vector<float>::iterator j = clip.begin(); j != clip.end(); ++j)
              mean += *j;
            mean /= clip.size();
            for (vector<float>::iterator j = clip.begin(); j != clip.end(); ++j)
            {
              *j -= mean;
              if (fabs(*j) > max_amp)
                max_amp = fabs(*j);
            }
            for (vector<float>::iterator j = clip.begin(); j != clip.end(); ++j)
              *j /= max_amp * 1.05;
            SF_INFO sf_info;
            sf_info.samplerate = audio_msg->sample_rate;
            sf_info.channels = numChannels;
            sf_info.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
	    
            std::sprintf(num, "%.06d", clip_num++);
	    std::string filename(subdir_ + filename_chunk_ + "_" + num +".wav");	    
	    char *fnamebuf(convertStringToChar(filename));
	    
            SNDFILE *sf = sf_open(fnamebuf, SFM_WRITE, &sf_info);
            ROS_ASSERT(sf);
            sf_write_float(sf, &clip[0], clip.size());
            sf_close(sf);
            printf("done\n");
          }
          break;
      }
    }
  }
private:
  int32_t  clip_num, clip_start, clip_end;
  uint64_t audio_clock;
  enum { IDLE, CLIP_START, IN_CLIP } clip_state;
  float window_power;
  deque<float> window;
  vector<float> clip;
  ros::Subscriber sub;
  ros::Subscriber control_sub_;
  ros::Subscriber current_sub_;
  std::string subdir_;
  std::string filename_chunk_;
  int run_status_;
  int current_stim_ind_;
  int cut_file_;
  ros::Publisher savefile_pub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "audio_clip_writer");
  ros::NodeHandle nh;

  AudioWriter aw(nh, argv[1]);
  cout << "About to spin" << endl;
  ros::spin();
  return 0;
}

