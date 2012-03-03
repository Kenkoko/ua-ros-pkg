#!/usr/bin/env python
import roslib
roslib.load_manifest('ultraspeech')
import rospy
import rosbag

import scikits.audiolab as audio
from numpy import array, reshape

import optparse

def write_clip(stimulus, stimtime, dirname, num_channels, sample_rate, data):
    format = audio.Format(type='au',encoding='float32')
    name = dirname + '/' + str(stimtime.secs) + '.' + str(stimtime.nsecs) + '_' + stimulus + '.au'
    print name
    soundfile = audio.Sndfile(name, 'w', format, num_channels, sample_rate)
    for d in data:
        sndarray = array(d)
        sndarray  = reshape(sndarray, (len(sndarray)/num_channels ,num_channels) )
        soundfile.write_frames(sndarray)
    soundfile.sync()

def exportBag(opts):
    bag = rosbag.Bag(opts.bagname)
    audio_buf = []
    time_before = 0
    sample_rate = None
    last_stim = None
    last_stim_time = None
    current_stim = None
    current_stim_time = None
    audio_chunks_since_last_stim = 0
    for topic, msg, t in bag.read_messages(topics=['/audio_capture/audio', '/current_stimulus']):
        if topic == '/audio_capture/audio':
            num_channels = msg.num_channels
            sample_rate = msg.sample_rate
            audio_buf.append(msg.samples)
            audio_chunks_since_last_stim += 1
            if not opts.singlefile:
                if (audio_chunks_since_last_stim == opts.time_after) and (last_stim is not None):
                    write_clip(last_stim, last_stim_time, opts.dirname, num_channels, sample_rate, audio_buf)
                    audio_buf = audio_buf[-(audio_chunks_since_last_stim+time_before):]
        if topic == '/current_stimulus':
            audio_chunks_since_last_stim = 0
            last_stim = current_stim
            last_stim_time = current_stim_time
            current_stim = msg.stimulus
            current_stim_time = t

    if current_stim is not None:
        if opts.singlefile:
            write_clip('sound', '', opts.dirname, num_channels, sample_rate, audio_buf)
        else:
            write_clip(current_stim, current_stim_time, opts.dirname, num_channels, sample_rate, audio_buf)

    bag.close()

if __name__ == "__main__":
    parser = optparse.OptionParser()
    parser.add_option('-b', '--bagfile', help='bagfile to read', dest='bagname')
    parser.add_option('-d', '--directory', help='path where new soundfiles will go', dest='dirname', default = '.')
    parser.add_option('-a', '--after', help='amount of audio after click to include', dest='time_after', type='int', default = 8)
    parser.add_option('-s', '--single-file', help='Don''t split sound into clips', dest='singlefile', default=False, action='store_true')
    (opts, args) = parser.parse_args()

    if opts.time_after < 1:
        opts.time_after = 1
    if opts.singlefile:
        print "Storing as a single file."
    if opts.bagname is None:
        print "bagfile option must be input on command line"
        parser.print_help()
        exit()


