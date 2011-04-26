#!/usr/bin/env python

import sys
import time
import socket
import threading
import os

import pygtk
pygtk.require('2.0')
import gtk

import roslib
roslib.load_manifest('game_faces')
import rospy

import numpy

# Import functions for starting the video thread
import shlex, subprocess

from logger import logger
import datetime

from game_faces.srv import *
from game_faces.msg import TwoPersonGame, GamePlay, GameSummary
from games import UltimatumGameController
from games import Welcome

from video_wrapper import videowrapper

#gtk.gdk.threads_init()

capture_video = 1

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class GFPlayer:
    def __init__(self, player_id=-1, is_first=False, game_topic=''):
        self.player_id = player_id
        self.is_first = is_first
        self.game_topic = game_topic
        self.game_topic_lock = threading.Lock()

class logger():
    def __init__(self, filename, condition, controller):
        self.condition = condition
        self.home = os.path.expanduser('~')
        self.controller = controller
        self.file = open(filename, 'w')
        self.temp = open(self.home + "/ros/ua-ros-pkg/ua_game_theory/Data/Info_Players/temp.txt", "w")
        if self.controller is not 'Experimenter':
            self.file.write(str(time.asctime( time.localtime(time.time()))) + " Condition: " + str(self.condition) + '\n')

    def close(self):
        self.file.close()

    def writer(self, data):
        self.file.write(data + '\n')

    def data_writer(self,num_player,player_role, offer, reaction, payment, condition, opponent):
        self.temp.write(str(num_player) + ' \t '  + str(player_role) + ' \t ' + str(offer) + ' \t ' + str(reaction) + ' \t ' + str(payment) + ' \t ' + str(condition) + '\t' + str(opponent) +'\n')

    def header(self):
        self.temp.write('Num Player \t Role \t Offer \t Reaction \t Payment \t Condition \t Opponent \n')

    def copy_data(self):
        self.temp.close()
        temp = open(self.home + "/ros/ua-ros-pkg/ua_game_theory/Data/Info_Players/temp.txt", "r")
        for line in temp.readlines():
            self.file.write(line)
        temp.close()

class GFConsoleController:
    def __init__(self):
        size = 16
        self.view = gtk.Frame()
        self.text_buffer = gtk.TextBuffer()
        self.text_buffer.create_tag('ozzy', size_points = size)
        self.text_buffer.create_tag('ozzy1', size_points = size, foreground = "blue")
        self.text_buffer.create_tag('ozzy2', size_points = size, foreground = "Green")
        self.text_buffer.create_tag('ozzy3', size_points = size, foreground = "Red")
        self.textview = gtk.TextView(self.text_buffer)
        self.textview.set_editable(False)
        self.textview.set_cursor_visible(False)
        self.textview.set_left_margin(10)
        self.scrollview = gtk.ScrolledWindow()
        self.scrollview.add_with_viewport(self.textview)
        self.scrollview.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.view.add(self.scrollview)
        self.view.show_all()

    def scroll_to_bottom(self):
        v_adj = self.scrollview.get_vadjustment()
        v_adj.set_value(v_adj.get_upper())
    
    def set_text(self, text):
        self.text_buffer.set_text(str(text))
        self.scroll_to_bottom()
        
    def append_text(self, text):
        self.text_buffer.insert(self.text_buffer.get_end_iter(), text)
        self.text_buffer.apply_tag_by_name('ozzy', self.text_buffer.get_start_iter(), self.text_buffer.get_end_iter())
        self.scroll_to_bottom()

    def append_text_instructions(self, text, condition):
        self.text_buffer.insert(self.text_buffer.get_end_iter(), text)
        if condition == 0:
            self.text_buffer.apply_tag_by_name('ozzy1', self.text_buffer.get_start_iter(), self.text_buffer.get_end_iter())
            self.scroll_to_bottom()
        elif condition == 1:
            self.text_buffer.apply_tag_by_name('ozzy2', self.text_buffer.get_start_iter(), self.text_buffer.get_end_iter())
            self.scroll_to_bottom()
        elif condition == 2:
            self.text_buffer.apply_tag_by_name('ozzy3', self.text_buffer.get_start_iter(), self.text_buffer.get_end_iter())
            self.scroll_to_bottom()
 
    def clear(self):
        self.text_buffer.set_text('')

class GameFacesUI:
    def __init__(self):
        # ivars
#        self.player = GFPlayer()
        self.the_game_controller = None
    
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("destroy", lambda w: gtk.main_quit())
        self.window.connect("delete_event", self.delete_event)
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")

        self.condition = 'nothing for now'

        self.home = os.path.expanduser('~')
        mch_name = os.path.split(self.home)[1]
        filenum = 0
        while True:
            self.filename = self.home + "/ros/ua-ros-pkg/ua_game_theory/Data/Info_Players/subject_" + mch_name +"_" + str(filenum) + ".txt"
            filenum += 1
            if not os.path.exists(self.filename): break
        self.log = logger(self.filename, self.condition, 'nothing for now')

        accelgroup = gtk.AccelGroup()
        key, modifier = gtk.accelerator_parse('Escape')
        accelgroup.connect_group(key, modifier, gtk.ACCEL_VISIBLE, gtk.main_quit)
        self.window.add_accel_group(accelgroup)

        self.window.resize(500, 500)
# The full screen constitutes problems...
#        self.window.fullscreen()

        self.content_vbox = gtk.VBox(False, 8)
        self.content_vbox.set_border_width(10)
        self.window.add(self.content_vbox)
        
        self.console = GFConsoleController()
        self.content_vbox.pack_end(self.console.view)

        self.totalpayoff = 0  
        self.results = []      
        self.last_game_type = ''

#        self.log = logger()

        self.window.show_all()
        self.make_instructions(self.window, self.console, self.content_vbox, self.filename, self.log, self.condition)
#        while self.condition == 'nothing for now':
#            time.sleep(1)


    def make_instructions(self, window, console, content_vbox, filename, log, condition):
        gts = GtkThreadSafe()
        with gts:
            self.the_game_controller = Welcome(window, console, content_vbox, filename, log, condition)
            self.content_vbox.pack_start(self.the_game_controller.view, expand=False)

    def delete_event(self, widget, event, data=None):
        print "Delete event called. Closing window."
        if capture_video:
            print 'boom'
            #self.video.terminate()
            #self.video.end_video()
        return False

if __name__ == "__main__":
    g = GameFacesUI()
    gtk.main()
