#!/usr/bin/env python

import roslib; roslib.load_manifest('ua_audio_capture')
from ua_audio_msgs.msg import AudioRawStream

import rospy
from threading import Lock
from std_srvs.srv import Empty

from array import array

class AudioDump():
    def __init__(self):
        self.save_audio = False
        self.stream = []
        self.counter = 0
        self.object_id = 0
        self.action_id = 0
        self.last_index = 0
        self.l = Lock()
        
        rospy.init_node('audio_dump', anonymous=True)
        
        time = rospy.Time.now().to_sec()
        self.f = open('/tmp/sound_%d.rawsound' % time, 'wb')
        self.fdesc = open('/tmp/sound_%d.desc' % time, 'w')
        
        rospy.Subscriber('audio_capture/audio', AudioRawStream, self.process_audio)
        rospy.Service('audio_dump/trigger', Empty, self.process_trigger)

    def __del__(self):
        self.f.close()
        self.fdesc.close()

    def process_audio(self, msg):
        self.l.acquire()
        if self.save_audio:
            self.stream.extend(msg.samples)
        self.l.release()

    def process_trigger(self, req):
        rospy.loginfo('Trigger #%d, save audio currently is %s, will set to %s' % (self.counter, str(self.save_audio), str(not self.save_audio)))
        
        self.l.acquire()
        self.save_audio = not self.save_audio
        
        # was recording, now stop and dump to file
        if not self.save_audio:
            # save the sound to file and ditch it from memory
            self.save_sound()
            self.stream = []
            
        self.l.release()
        
        self.counter += 1
        return []   # success

    def save_sound(self):
        stream_array = array('d', self.stream)
        stream_array.tofile(self.f)
        print 'current stream has %d numbers' % len(self.stream)
        offset = self.last_index
        self.last_index += len(self.stream)
        print 'current sound begins at %d and next sound will start at %d' % (offset, self.last_index)
        desc = '%d %d %d %d\n' % (self.object_id, self.action_id, offset, len(self.stream))
        self.fdesc.write(desc)
        
        #self.object_id += 1
        #self.f.write(str(self.stream))

if __name__ == '__main__':
    a = AudioDump()
    rospy.spin()

