#!/usr/bin/env python
import roslib
roslib.load_manifest('ultraspeech')
import rospy
import rosbag
import sys

import sunau

def write_clip(stimulus,num_channels, sample_rate, data):
    soundfile = sunau.open(stimulus + '.au', 'w')
    soundfile.setnchannels(num_channels)
    soundfile.setframerate(sample_rate)
    soundfile.setsampwidth(4)
    soundfile.writeframes(data)
    soundfile.close()
    
if __name__ == "__main__":
    bagname = 'test.bag'
    if len(sys.argv) > 1:
        bagname = sys.argv[1]    
    bag = rosbag.Bag(bagname)
    
    # Buffer of K packets before and after a stim
    K = 20
    audio_buf = []
    sample_rate = None
    last_stim = None
    current_stim = None
    audio_chunks_since_last_stim = 0
    for topic, msg, t in bag.read_messages(topics=['/audio_capture/audio', '/current_stimulus']):
        print t
        print topic
        if topic == '/audio_capture/audio':
            num_channels = msg.num_channels
            sample_rate = msg.sample_rate
            audio_buf.append(msg.samples)
            audio_chunks_since_last_stim += 1
            if (audio_chunks_since_last_stim == K) and (last_stim is not None):
                write_clip(last_stim, num_channels, sample_rate, audio_buf)
                audio_buf = audio_buf[len(audio_buf)-audio_chunks_since_last_stim-K:]
        if topic == '/current_stimulus':
            audio_chunks_since_last_stim = 0
            last_stim = current_stim
            current_stim = msg.stimulus
    
    if current_stim is not None:
        write_clip(current_stim, num_channels, sample_rate, audio_buf)

    bag.close()
