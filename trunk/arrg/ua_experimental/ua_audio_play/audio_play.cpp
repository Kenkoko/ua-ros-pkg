///////////////////////////////////////////////////////////////////////////////
// A simple audio player which uses PortAudio to play an audio stream from ROS
//
// Copyright (C) 2010, Morgan Quigley
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

#include <queue>
#include <ros/ros.h>
#include <audio_msgs/AudioRawStream.h>
#include "portaudio.h"

std::deque<audio_msgs::AudioRawStream::ConstPtr> g_segment_queue;

struct AudioStreamState {
  audio_msgs::AudioRawStream::ConstPtr current_segment;
  unsigned int index;

  AudioStreamState() : index(0) {}
};

static int pa_audio_cb(const void *input_buffer, void *output_buffer,
                       unsigned long frames_per_buffer,
                       const PaStreamCallbackTimeInfo *time_info,
                       PaStreamCallbackFlags status, void *user_data)
{
  float *out = (float *)output_buffer;

  AudioStreamState *state = (AudioStreamState*)user_data;
  for (unsigned long i = 0; i < frames_per_buffer; i++)
  {
    if (!state->current_segment || state->index >= state->current_segment->samples.size())
    {
      if (g_segment_queue.empty())
        state->current_segment.reset();
      else
      {
        state->current_segment = g_segment_queue.front();
        g_segment_queue.pop_front();
      }
      state->index = 0;
    }

    if (state->current_segment)
      *out++ = state->current_segment->samples[state->index++];
    else
      *out++ = 0.0;
  }

  return paContinue;
}

void ros_audio_cb(const audio_msgs::AudioRawStream::ConstPtr &msg)
{
  g_segment_queue.push_back(msg);
}

#define PA_ERROR_CHECK(expr) do { PaError err = expr; if (err != paNoError) { ROS_FATAL("portaudio error: %s", Pa_GetErrorText(err)); ROS_BREAK(); } } while(0)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "audio_play");
  ros::NodeHandle n;
  ros::Subscriber audio_sub = n.subscribe("audio_capture/audio", 10, ros_audio_cb);

  AudioStreamState audio_stream_state;

  PaStream *stream;

  PA_ERROR_CHECK( Pa_Initialize() );
  PA_ERROR_CHECK( Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, 44100, 4096, pa_audio_cb, &audio_stream_state) );
  PA_ERROR_CHECK( Pa_StartStream(stream) );

  ros::spin();

  ROS_INFO("shutting down portaudio...");
  PA_ERROR_CHECK( Pa_StopStream(stream) );
  PA_ERROR_CHECK( Pa_CloseStream(stream) );
  Pa_Terminate();
  ROS_INFO("done shutting down portaudio. good night.");

  return 0;
}

