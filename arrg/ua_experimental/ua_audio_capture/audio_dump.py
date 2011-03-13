#!/usr/bin/env python

# Author: Antons Rebguns

import roslib; roslib.load_manifest('ua_audio_capture')
from ua_audio_msgs.msg import AudioRawStream

import rospy
from threading import Lock
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording
from ua_audio_capture.srv import StopAudioRecordingResponse

from array import array

class AudioDump():
    def __init__(self):
        self.save_audio = False
        self.stream = []
        self.counter = 1
        self.object_id = 0
        self.action_id = 0
        self.last_index = 0
        self.l = Lock()
        
        rospy.init_node('audio_dump', anonymous=True)
        
        time = rospy.Time.now().to_sec()
        self.f = open('/tmp/sound_%d.rawsound' % time, 'wb')
        self.fdesc = open('/tmp/sound_%d.desc' % time, 'w')
        
        rospy.Subscriber('audio_capture/audio', AudioRawStream, self.process_audio)
        rospy.Service('audio_dump/start_audio_recording', StartAudioRecording, self.process_start_recording)
        rospy.Service('audio_dump/stop_audio_recording', StopAudioRecording, self.process_stop_recording)

    def __del__(self):
        self.f.close()
        self.fdesc.close()

    def process_audio(self, msg):
        if self.save_audio:
            self.l.acquire()
            self.stream.extend(msg.samples)
            self.l.release()

    def process_start_recording(self, req):
        rospy.loginfo('Recording sound #%d' % self.counter)
        
        self.l.acquire()
        self.action_id = req.action_id
        self.object_id = req.object_id
        self.save_audio = True
        self.l.release()
        
        return [] # success

    def process_stop_recording(self, req):
        rospy.loginfo('Stopping recording of sound sample #%d' % self.counter)
        res = StopAudioRecordingResponse()
        
        # was recording, now stop and dump to file
        if self.save_audio:
            # save the sound to file and ditch it from memory
            self.save_audio = False
            
            self.l.acquire()
            res.recorded_sound = self.stream
            self.stream = []
            self.l.release()
            
            if req.dump_to_file:
                rospy.loginfo('Save current sound sample to file')
                self.save_sound(res.recorded_sound)
            else:
                rospy.logwarn('Will not save current sound sample to file')
        else:
            rospy.logwarn('Was not recording audio, will return empty array')
            
        self.counter += 1
        return res

    def save_sound(self, sound):
        stream_array = array('d', sound)
        stream_array.tofile(self.f)
        rospy.loginfo('current stream has %d numbers' % len(sound))
        offset = self.last_index
        self.last_index += len(sound)
        rospy.loginfo('current sound begins at %d and next sound will start at %d' % (offset, self.last_index))
        desc = '%d %d %d %d\n' % (self.action_id, self.object_id, offset, len(sound))
        self.fdesc.write(desc)
        self.fdesc.flush()

if __name__ == '__main__':
    a = AudioDump()
    rospy.spin()

