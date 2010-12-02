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
#include <deque>
#include <vector>
#include "ros/ros.h"
#include "ua_audio_msgs/AudioRawStream.h"
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

class AudioWriter
{
public:
  AudioWriter(ros::NodeHandle nh)
  : clip_num(0), clip_start(0), clip_end(0), audio_clock(0),
    clip_state(IDLE), window_power(0)
  {
    sub = nh.subscribe<ua_audio_msgs::AudioRawStream>("audio_capture/audio", 5, boost::bind(&AudioWriter::audio_cb, this, _1));
    cout << "Existing" << endl;
  }
  void audio_cb(ua_audio_msgs::AudioRawStreamConstPtr audio_msg)
  {
    //if (audio_msg->num_channels != 1)
    //{
    //  ROS_FATAL("audio_clip_writer can only handle single-channel audio.");
    //  ROS_BREAK();
    //}
    uint32_t numChannels = audio_msg->num_channels;
    uint32_t numSamples = audio_msg->samples.size() / numChannels;
    //cout << "numChannels: " << numChannels << endl;
    for (size_t i = 0; i < numSamples; i++)
    {
      audio_clock++;
      // cout << "Audio clock: " << audio_clock << endl;
      // maintain a window of 0.5 seconds
      if (window.size() >= numChannels * WINDOW_LENGTH * audio_msg->sample_rate){
          for(uint32_t j=0; j < numChannels; ++j){        
              window.pop_front();
          }
      }
      for(uint32_t j=0; j < numChannels; ++j){        
            window.push_back(audio_msg->samples[i*numChannels+j]);
            //printf("%f, ", audio_msg->samples[i*numChannels+j]);
      }
      //cout << endl << "--" << endl;
      if (audio_clock % 100 == 0)
      {
        // compute RMS power
        float sum_squares = 0;
        int win_idx = 0;
        for (deque<float>::iterator j = window.begin(); j != window.end();
             ++j, win_idx++)
          sum_squares += (*j) * (*j);// * ((float)win_idx / window.size());
        //window_power = sqrt(2 * sum_squares / window.size());
        window_power = sqrt(sum_squares / window.size() );
        //printf("power: %f\n", window_power);
      }
      switch (clip_state)
      {
        case IDLE:
          if (fabs(audio_msg->samples[i* numChannels]) > CLIP_START_POWER)
          {
            printf("clip start\n");
            clip_start = audio_clock;
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
            if (window_power > CLIP_MAINTAIN_POWER)
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
          if (window_power < CLIP_MAINTAIN_POWER)
          {
            clip_state = IDLE;
            const float clip_len = (float)clip.size() / audio_msg->sample_rate / numChannels;
            if (clip_len >= MIN_CLIP_LENGTH)
            {
              printf("good clip, length = %.2f seconds\n", clip_len);
              // normalize it
              float max_amp = 0, mean = 0;
              for (vector<float>::iterator j = clip.begin();
                   j != clip.end(); ++j)
                mean += *j;
              mean /= clip.size();
              for (vector<float>::iterator j = clip.begin();
                   j != clip.end(); ++j)
              {
                *j -= mean;
                if (fabs(*j) > max_amp)
                  max_amp = fabs(*j);
              }
              for (vector<float>::iterator j = clip.begin();
                   j != clip.end(); ++j)
                *j /= max_amp * 1.05;
              SF_INFO sf_info;
              sf_info.samplerate = audio_msg->sample_rate;
              sf_info.channels = numChannels;
              sf_info.format = SF_FORMAT_AU | SF_FORMAT_FLOAT;
              char fnamebuf[100];
              snprintf(fnamebuf, sizeof(fnamebuf), "clip%06d.au", clip_num++);
              SNDFILE *sf = sf_open(fnamebuf, SFM_WRITE, &sf_info);
              ROS_ASSERT(sf);
              sf_write_float(sf, &clip[0], clip.size());
              sf_close(sf);
              //char cmd[500];
              //system(cmd);
              printf("done\n");
            }
            else
              printf("too short. length = %.2f seconds\n", clip_len);
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "audio_clip_writer");
  ros::NodeHandle nh;
  AudioWriter aw(nh);
  cout << "About to spin" << endl;
  ros::spin();
  //while (nh.ok())
    //ros::Duration(0, 500000000).sleep();
  return 0;
}

