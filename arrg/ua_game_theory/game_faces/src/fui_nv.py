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
from games import PaymentScreen

from video_wrapper import videowrapper

gtk.gdk.threads_init()

capture_video = 0

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

class GFConsoleController:
    def __init__(self):
        self.view = gtk.Frame()
        self.text_buffer = gtk.TextBuffer()
        self.textview = gtk.TextView(self.text_buffer)
        self.textview.set_editable(False)
        self.textview.set_cursor_visible(False)
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
        self.scroll_to_bottom()
        
    def clear(self):
        gts = GtkThreadSafe()
        with gts:
            self.text_buffer.set_text('')

class GameFacesUI:
    def __init__(self):
        # ivars
        self.player = GFPlayer()
        self.the_game_controller = None
    
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("destroy", lambda w: gtk.main_quit())
        self.window.connect("delete_event", self.delete_event)
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")
# The full screen constitutes problems...
#        self.window.fullscreen()
        accelgroup = gtk.AccelGroup()
        key, modifier = gtk.accelerator_parse('Escape')
        accelgroup.connect_group(key, modifier, gtk.ACCEL_VISIBLE, gtk.main_quit)
        self.window.add_accel_group(accelgroup)
        self.window.resize(500, 500)

        self.content_vbox = gtk.VBox(False, 8)
        self.content_vbox.set_border_width(10)
        self.window.add(self.content_vbox)
        
        self.console = GFConsoleController()
        self.content_vbox.pack_end(self.console.view)

        self.totalpayoff = 0  
        self.results = []      
        self.last_game_type = ''

        self.log = logger()

        self.window.show_all()
        self.register_player()
        
    def register_player(self):
        self.console.append_text("Connecting to Game Master...\n")
        rospy.init_node('game_faces_ui', anonymous=True)
        self.game_server_sub = rospy.Subscriber("game_master", TwoPersonGame, self.play_game)

        rospy.wait_for_service('register_game_player')
        try:
            reg = rospy.ServiceProxy('register_game_player', RegisterGamePlayer)
            response = reg(str(roslib.network.get_host_name()), 1)
            self.player.player_id = response.player_id
        except rospy.ServiceException, e:
            print "Could not get a player_id from master"
            sys.exit(1)

        # Set up video capture
        video_topic = "Video" + str(self.player.player_id)
        self.video_pub = rospy.Publisher(video_topic, TwoPersonGame)
        
        # Using rosrun spawns two processes, and the second does not terminate correctly 
        #run_video_command = 'rosrun game_faces game_video_capture ' + str(self.player.player_id)
        
        #run_video_command = '../bin/game_video_capture ' + str(self.player.player_id)
        if capture_video:
            #args = shlex.split(run_video_command)
            #self.video = subprocess.Popen(args)
            self.video = videowrapper(self.player.player_id)
            self.video.start_video()

        # Start a log for this player
        home = os.path.expanduser('~')
        filename = home + "/ros/ua-ros-pkg/ua_game_theory/Data/Info_Players/playerlog_" + str(self.player.player_id) + "_" + str(datetime.date.today()) + ".txt"
        self.log.openlog(filename)

        self.console.append_text("Ready to play.\nWaiting for your opponent...\n")


    def play_game(self, gamedata):
        gts = GtkThreadSafe()
        stopvideo = False
        if gamedata.game_type == 'NO_MORE_GAMES':
            self.console.append_text('\n\nNo more games!\n')
            with gts:
                 self.the_game_controller = PaymentScreen(self.window, self.console, self.player, self.results, self.log)
                 self.content_vbox.pack_start(self.the_game_controller.view, expand=False)

            if capture_video:
                self.video.end_video()
                #self.video.terminate()  # is this the correct way to terminate video?
            return
        
        self.player.game_topic_lock.acquire()
        if self.player.game_topic is '':
            is_first_player = (gamedata.first_player == self.player.player_id)
            is_second_player = (gamedata.second_player == self.player.player_id)
            if is_first_player or is_second_player:
                if self.the_game_controller:
                    try:
                        if self.the_game_controller.isgame:
                            self.results.append( (self.last_game_type, self.the_game_controller.payoff) )
                            self.totalpayoff += self.the_game_controller.payoff
                            smsg = GameSummary()
                            smsg.player_id = self.player.player_id
                            smsg.payoff = self.the_game_controller.payoff
                            self.summary_pub.publish(smsg)
                            self.summary_pub.unregister()
                            print "Player's total payoff:", self.totalpayoff
                            print "Results:", self.results
                        else:
                            print "not a game"
                    except AttributeError:  # Non-game objects will not have an 'isgame' member
                        print "No payoff for this round"
                        smsg = GameSummary()
                        smsg.player_id = self.player.player_id
                        smsg.payoff = 0
                        self.summary_pub.publish(smsg)
                        self.summary_pub.unregister()
                            
                    self.content_vbox.remove(self.the_game_controller.view)
                self.console.clear()
                self.console.append_text("\nStarting game: %s\n\n\n" %gamedata.game_type)
                
                # Set up a publisher for the summary at game end
                self.summary_pub = rospy.Publisher(gamedata.game_topic+"s", GameSummary)
                
                # Register the game playing topic
                game_topic = gamedata.game_topic
                print game_topic
                self.player.is_first = is_first_player
                print "1st player? ", is_first_player
                self.player.game_topic = game_topic
                game_type = str(gamedata.game_type)
                print "Starting: ", game_type
                if game_type == "Ultimatum":
                    with gts:
			if capture_video:
				self.the_game_controller = UltimatumGameController(self.window, self.console, self.player, self.log, self.video)
		                self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
			else:
		                self.the_game_controller = UltimatumGameController(self.window, self.console, self.player, self.log)
		                self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                elif game_type == "PaymentScreen":
#                elif game_type == "NO_MORE_GAMES":
                    with gts:
                       self.the_game_controller = PaymentScreen(self.window, self.console, self.player, self.results, self.log)
                       self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                       stopvideo = True
#                elif game_type == "NO_MORE_GAMES":
#                    with gts:
#                        self.the_game_controller = ThanksController(self.window, self.console, self.player)
#                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                else:
                    with gts:
                        print game_type
                        print "Didnt match game_type: %s" %game_type
                        sys.exit(1)
                
                if capture_video:
                    self.video_pub.publish(gamedata)
                        
                self.last_game_type = game_type
                self.the_game_controller.take_first_turn()
                # after this, get the results of the game from the controller
                
                # Maybe a better way to do this
                if stopvideo:
                    time.sleep(5) # Grab 5 more seconds
                    if capture_video:
                        self.video.end_video()
            
        else:
            with gts:
                print "skipped a game message from master, because I'm already in a game. Topic: ", self.player.game_topic
        self.player.game_topic_lock.release()

    def delete_event(self, widget, event, data=None):
        print "Delete event called. Closing window."
        if capture_video:
            #self.video.terminate()
            self.video.end_video()
        return False

if __name__ == "__main__":
    g = GameFacesUI()
    gtk.main()
