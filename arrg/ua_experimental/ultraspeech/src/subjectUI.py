#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, datetime, random, math, time, subprocess, signal

import pygtk
pygtk.require('2.0')
import gtk
import gtk.glade
import pango

import roslib
roslib.load_manifest('ultraspeech')
import rospy

from ultraspeech.msg import CurrentStim, Control
from std_msgs.msg import String, UInt8

gtk.gdk.threads_init()

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class GFConsoleController:
    def __init__(self):
        self.view = gtk.Frame()
        texttagtable = gtk.TextTagTable()
        self.texttag = gtk.TextTag(None)
        self.texttag.set_property("weight", pango.WEIGHT_BOLD)
        self.texttag.set_property("size-points", 36)
        self.texttag.set_property("justification", gtk.JUSTIFY_CENTER)
        texttagtable.add(self.texttag)

        self.text_buffer = gtk.TextBuffer(texttagtable)        
        self.textview = gtk.TextView(self.text_buffer)
        self.textview.set_editable(False)
        self.textview.set_cursor_visible(False)
        self.textview.set_wrap_mode(gtk.WRAP_WORD)
        self.scrollview = gtk.ScrolledWindow()
        self.scrollview.add_with_viewport(self.textview)
        self.scrollview.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        
        self.view.add(self.scrollview)        
        self.view.show_all()

    def scroll_to_bottom(self):
        v_adj = self.scrollview.get_vadjustment()
        v_adj.set_value(v_adj.get_upper())

    def apply_tag(self):
        start = self.text_buffer.get_start_iter()
        end = self.text_buffer.get_end_iter()
        self.text_buffer.apply_tag(self.texttag, start, end)

    def set_text(self, text):
        self.text_buffer.set_text(str(text))
        self.apply_tag()
        
    def append_text(self, text):
        self.text_buffer.insert(self.text_buffer.get_end_iter(), text)
        self.apply_tag()
        self.scroll_to_bottom()
        
    def clear(self):
        self.text_buffer.set_text('')

class SubjectUI:
    def __init__(self):
        '''
        stimuliFile should be a list of words or sentences (1 per line) that will be randomized and displayed.
        numReps is the number of repitions for each word/sentence
        '''
        
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("destroy", self.onDestroy)
        self.window.set_title("Experiment")
        self.window.resize(1200, 500)

        self.content_vbox = gtk.VBox(False, 8)
        self.content_vbox.set_border_width(10)
        self.window.add(self.content_vbox)
                
        self.console = GFConsoleController()
        self.content_vbox.pack_start(self.console.view)
        
        self.button_hbox = gtk.HBox(False, 3)
        self.button_hbox.set_border_width(10)
        self.content_vbox.pack_start(self.button_hbox, expand=False)
        
        self.start_button = gtk.Button("_Start", None)
        self.quit_button = gtk.Button("S_top", None)
        self.next_button = gtk.Button("Forward", gtk.STOCK_GO_FORWARD)
        
        self.button_hbox.pack_start(self.start_button, fill=False)
        self.button_hbox.pack_start(self.quit_button, fill=False)
        self.button_hbox.pack_start(self.next_button, fill=False)
        
        self.start_button.connect("clicked", self.onStart)
        self.quit_button.connect("clicked", self.onStop)
        self.next_button.connect("clicked", self.onNext)
        
        self.window.show_all()

        gts = GtkThreadSafe()
        with gts:
            s = Startup()
            result, stimuliFile, numReps = s.run()
            if result == 1:
                # Control topic tells DV when to start and stop
                # current_stimulus signals what the subject is seeing when
                rospy.init_node('subjectUI')
                self.controlTopic = rospy.Publisher('control',Control)
                self.currentStimulusTopic = rospy.Publisher('current_stimulus', CurrentStim)
                self.createTopLevel(stimuliFile, numReps)
                self.startROSbag()
                self.run = 0
                self.currentInd = 0
                self.currentRep = 1
                self.currentBatch = 1
                self.total = len(self.stimuliList)
            else:
                 gtk.main_quit()
   
    def startROSbag(self):
        bagmsg = ['rosbag', 'record', 
                  '/control',
                  '/current_stimulus',
                  '/audio_capture/audio',
                  '/ultrasound_dvgrab/framenum',
                  '/camera/rgb/camera_info', '/camera/rgb/image_rect_color','/camera/depth_registered/image_rect',
                  '/rosout',
                  '--output-name=' + self.topleveldir + '/capture_all.bag']
        self.rosbag_pid = subprocess.Popen(bagmsg)
      
    def createTopLevel(self, stimuliFile, numReps):
        '''Creates the top level directory, randomizes input list'''
        self.sessionID = str(rospy.Time.now())
        self.topleveldir = os.environ['HOME'] + '/UltraspeechData/' + str(datetime.date.today()) + '_' + self.sessionID
        os.makedirs(self.topleveldir)        
        stimuli = open(stimuliFile, 'r').readlines()
        random.shuffle(stimuli)
        
        self.stimuliList = []
        for i in range(numReps):
	    for j in stimuli:
                if j != '\n':
                    self.stimuliList.append(j[:-1])
        
        self.numReps = numReps
        self.numBatches = int(math.ceil(float(len(self.stimuliList)) / 10.0))
        
        self.console.set_text("Press Start to begin")
    
    def onStart(self, event):
        # This is when the button is pressed
        self.controlStart(event)
      
    def controlStart(self, event):
        # This is what you actually do when you start
        # Reason: Need to separate button pushing logic from internal shutdown logic
        if self.run == 0:
            self.run = 1
            controlMessage = Control(top_level_directory=self.topleveldir,run=self.run)
            self.controlTopic.publish(controlMessage)
            self.onNext(event)

        
    def onStop(self, event):
        # This is when the button is pressed
        self.controlStop(event)

    def controlStop(self, event):
        # This is what you actually do when you stop
        # Reason: Need to separate button pushing logic from internal shutdown logic
        if self.run == 1:
            self.run = 0
            controlMessage = Control(top_level_directory=self.topleveldir,run=self.run)
            self.controlTopic.publish(controlMessage)

    def onNext(self, event):
        if self.run == 1:
            if self.currentInd == self.total:
                self.console.set_text("You have reached the\nend of the experiment") 
                self.controlStop(event)
            else:
                if (((self.currentInd)%(self.total/self.numReps)) == 0):
                    self.currentRep += 1
                #if (((self.currentInd)%10) == 0):
                #    self.currentBatch += 1
                #    self.controlStop(event)
                #    time.sleep(1)
                #    self.controlStart(event)
                
                print "Stim:", self.stimuliList[self.currentInd]
                self.console.set_text('\n'+ self.stimuliList[self.currentInd])
                
                currentMessage = CurrentStim(stimulus=self.stimuliList[self.currentInd],
                rep=self.currentRep, batch=self.currentBatch)
                currentMessage.header.stamp = rospy.Time.now()
                self.currentStimulusTopic.publish(currentMessage)
                self.currentInd += 1

    def onDestroy(self, event):
        if self.run == 1:
            self.run = 0
            controlMessage = Control(run=self.run)
            self.controlTopic.publish(controlMessage)
        self.rosbag_pid.send_signal(signal.SIGINT)
        gtk.main_quit()

class Startup:
    def __init__(self):
        self.gladefile = os.environ['HOME'] + "/ros/ua-ros-pkg/ua_experimental/ultraspeech/src/startup.glade" #this is a bad hack
    
    def run(self):
        self.wTree = gtk.glade.XML(self.gladefile, "dialog1")
        self.dlg = self.wTree.get_widget("dialog1")
        dic = { "on_button1_clicked" : self.onOpen}
        self.wTree.signal_autoconnect(dic)
    
        self.file_display = self.wTree.get_widget("entry1")
        self.reps_display = self.wTree.get_widget("entry2")
    
        result = self.dlg.run()
    
        stimuliFile = self.file_display.get_text()
        numReps = int(self.reps_display.get_text())
    
        self.dlg.destroy()
        return result, stimuliFile, numReps
    
    def onOpen(self, event):
        fc = gtk.FileChooserDialog(title='Open Stimuli File', parent=None, 
                                   action=gtk.FILE_CHOOSER_ACTION_OPEN, 
                                   buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, 
                                            gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        g_directory = fc.get_current_folder()
        fc.set_current_folder(g_directory)
        fc.set_default_response(gtk.RESPONSE_OK)
        fc.set_select_multiple(False)
        ffilter = gtk.FileFilter()
        ffilter.set_name('.txt Files')
        ffilter.add_pattern('*.txt')
        fc.add_filter(ffilter)
        response = fc.run()
        if response == gtk.RESPONSE_OK:
            self.datafile = fc.get_filename()
            g_directory = fc.get_current_folder()
            self.file_display.set_text(self.datafile)
        fc.destroy()
    
        
def usage():
    return "usage: rosrun ultraspeech subjectUI.py stimuliFile numReps"

if __name__ == "__main__":
    #Startup()
    g = SubjectUI()
    gtk.main()
