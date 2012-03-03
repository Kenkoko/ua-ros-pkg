#!/usr/bin/env python

# Take a session and clip file, and output ultrasound frames from DV file
# clip file has following format:
# soundfile start1-stop1, start2-stop2, ...
# where times are in the format hrs:mins:secs.hundredths
# We assume soundfile is of the form stimulustimestamp
import errno

import roslib
roslib.load_manifest('ultraspeech')
import rospy
import rosbag

import cv

import glob
import optparse
import os
import re


def getDVframes(dvfile, output_dir, frame_start, frame_end):
    """
    Snip out the selected frames from the DV file
    """
    capture = cv.CaptureFromFile(dvfile)
    #  print "Dimensions: ", cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_WIDTH),\
    "x", cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_HEIGHT)
    numFrames = cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_COUNT)
    # print "Num frames: ", numFrames

    for i in range(frame_start, frame_end):
        print "Exporting frame", i
        cv.SetCaptureProperty(capture, cv.CV_CAP_PROP_POS_FRAMES, i)
        img = cv.RetrieveFrame(capture)
        cv.SaveImage(os.path.join(output_dir, 'frame' + str(i)) + '.png', img)
        #cv.ShowImage("Frame " + str(i), img)
        #cv.WaitKey()


def timecode2ROSTime(timecode):
    """
    Timecode in the form DD:HH:MM:SS.ss
    """
    splits = timecode.split(':')
    secs = float(splits[-1])
    if len(splits) > 1:
        secs += float(splits[-2])*60
    if len(splits) > 2:
        secs += float(splits[-3])*3600
    if len(splits) > 3:
        secs += float(splits[-4])*3600*24
    return rospy.Time(secs)



def parseClips(clipfile):
    """
    Extract the times to grab as ROSTime objects
    """
    clips = []
    f = open(clipfile, 'r')
    for line in f:
        # First find the digits that start the seconds part of the rostime
        m = re.match("[0-9]+.",line)
        secs =int(m.group(0)[:-1])
        # Then find the digits that start the naonseconds part of the rostime
        nsecs = int(re.match("[0-9]+",line[m.end():]).group(0))
        # Turn these into a ROS Time object
        clip_time = rospy.Time(secs,nsecs)

        # Now match all start-stop pairs in the line
        snippets = re.findall("[0-9:.]+\s*-\s*[0-9:.]+", line)
        for snippet in snippets:
            print "snippet",snippet
            start = re.match("[0-9:.]+\s*-",snippet)
            start_time = timecode2ROSTime(start.group(0)[:-1])
            end_time = timecode2ROSTime(snippet[start.end():])
            duration_after_clip_start = rospy.Duration(start_time.secs, start_time.nsecs)
            clips.append( (clip_time + duration_after_clip_start, end_time - start_time) )
    for t in clips:
        print t

    return clips

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST:
            pass
        else: raise

if __name__ == "__main__":
    usage = "usage: %prog [options] clipfile"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('-i', '--input-directory', help='Directory containing bagfiles to have snippets extracted', dest='subject_dir', default = '.')
    parser.add_option('-o', '--output-directory', help='Directory where new Ultrasound snippets will be saved', dest='output_dir', default = '.')
    (opts, args) = parser.parse_args()
    clipfile = args[0]

    bags = glob.glob( os.path.join( opts.subject_dir, '*.bag') )
    if len(bags) is 0:
        print "No bagfiles found"
        exit()

    clips = parseClips(clipfile)

    dvfile = ''
    dirname = ''
    clips_done = False
    clipiter = iter(clips)
    clip = clipiter.next() # fetch first value
    start_time = clip[0]
    end_time = clip[0] + clip[1]
    for infile in bags:
        print "current file is: " + infile
        shortpath = os.path.basename(infile)
        dirname = os.path.dirname(infile)
        print "dirname,  shortpath", dirname, shortpath
        bag = rosbag.Bag(infile)
        framelist = []
        start_frame = None
        end_frame = None
        for topic, msg, t in bag.read_messages(topics=['/ros_dvgrab/framenum', '/control']):
            #print topic, msg, t
            if topic == '/control':
                ultrasound_filename = msg.ultrasound_filename
                ultrasound_filename = os.path.basename(ultrasound_filename)
                root, ext = os.path.splitext(ultrasound_filename)
                dvfiles = glob.glob( os.path.join(dirname, root + '*.dv') )
                dvfile = dvfiles[0]
                print "dvfile", dvfile
            elif topic == '/ros_dvgrab/framenum':
                #print "start_time, end_time, t,", start_time, end_time, t
                if t <= start_time:
                    #print "t: ", t, ", start_time", start_time, "t <= start_time"
                    start_frame = msg
                if t > end_time:
                    end_frame = msg
                    #print "start_frame,", start_frame, "end_frame,", end_frame
                    clipdir = os.path.join(opts.output_dir, str(start_time.secs) + str(start_time.nsecs))
                    mkdir_p(clipdir)
                    getDVframes(dvfile, clipdir, start_frame.data, end_frame.data)
                    try: clip = clipiter.next() # fetch first value
                    except StopIteration: clips_done = True
                    start_time = clip[0]
                    end_time = clip[0] + clip[1]
                    start_frame = msg
            if clips_done:
                break
        if clips_done:
            break


