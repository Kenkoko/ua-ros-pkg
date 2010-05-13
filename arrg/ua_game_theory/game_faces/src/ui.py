#!/usr/bin/env python

import sys
import time
import socket
import threading

import pygtk
pygtk.require('2.0')
import gtk

import roslib
roslib.load_manifest('game_faces')
import rospy

from game_faces.srv import *
from game_faces.msg import TwoPersonGame, GamePlay

gtk.gdk.threads_init()

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class RosTopicProxy:
    def __init__(self, topic, msg_type, callback):
        self.sub = None
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
    
    def connect(self):
        if not rospy.core.is_initialized():
            # only called once per execution
            try:
                print roslib.scriptutil.get_master().getPid('/')
            except socket.error:
                print "Unable to communicate with master!"
            else:
                rospy.init_node('ui_topic_proxy', anonymous=True)
        self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)
    
    def disconnect(self):
        if self.sub:
            self.sub.unregister()
    
    def __del__(self):
        self.disconnect()

class GFPlayer:
    def __init__(self, player_id, is_first, game_topic):
        self.player_id = player_id
        self.is_first = is_first
        self.game_topic = game_topic
        self.game_topic_lock = threading.Lock()

class UltimatumGame:
    def __init__(self, shared_console, player):
        self.shared_console = shared_console
        self.player = player
        
        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(not self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.title = "Ultimatum"
        self.frame = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.frame.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)
        
        self.label_balance_desc = gtk.Label('Your current balance is')
        self.box_status.add(self.label_balance_desc)
        
        self.label_balance_amt = gtk.Label()
        self.set_balance(0)
        self.box_status.add(self.label_balance_amt)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)
        
        self.box_control.add(gtk.Label('Amount to give'))
        
        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=1, upper=10, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)
        
        self.frame.show_all()
        self.toggle_user_interaction()
    
    def ok_button_clicked(self, widget, data=None):
        self.shared_console.append_text('hello world\n')
    
    def set_balance(self, balance):
        self.label_balance_amt.set_markup('<span size="20000" weight="bold">%d</span>' %balance)
    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)
    
    def take_first_turn(self):
        if not self.player.is_first:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):
        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first:
                with gts:
                    self.set_balance(10)
                    self.toggle_user_interaction()
                    self.shared_console.append_text("Choose an amount, between 0 and 10 and press OK\n")
#                gp = GamePlay(play_number=1,amount=offer, player_id=self.player_id)
#                gp.header.stamp = rospy.Time.now()
#                self.play_pub.publish(gp)
#                print "Waiting for second player"
            else:
                with gts:
                    self.shared_console.append_text("Waiting for first player...\n")
#        elif play_number == 1:
#            if self.player.is_first_player:
#                pass    
#            else:
#                print "Player 1 sent "+str(game_play.amount)
#                print "You can either accept or reject this offer"
#                choice = raw_input("Please, input 'A' to accept or 'R' to reject the offer")
#                if choice == "A":
#                    self.current_amount = str(game_play.amount)
#                else:
#				    self.current_amount = 0
#                print "Your payoff is now: " + str(self.current_amount)
#                gp = GamePlay(play_number=2,amount=ord(choice), player_id=self.player_id)
#                gp.header.stamp = rospy.Time.now()
#                self.play_pub.publish(gp)
#        elif play_number == 2:
#            if self.player.is_first_player:
#                choice = chr(game_play.amount)
#                if choice == 'A':
#                    which_choice = "Accepted"
#                    self.current_amount = self.original - self.offer
#                else:
#                    which_choice = "Rejected"
#                    self.current_amount = 0
#                print "The other player decide to" + str(which_choice) + "your offer"
#                print "Your payoff for this round is:" + str(self.current_amount)
#                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
#                gp.header.stamp = rospy.Time.now(); self.play_pub.publish(gp)  
#            else:
#                pass
#            print "Please wait for the next game to start"
#            self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()

class GFConsole:
    def __init__(self):
        self.frame = gtk.Frame()
        self.text_buffer = gtk.TextBuffer()
        self.textview = gtk.TextView(self.text_buffer)
        self.textview.set_editable(False)
        self.textview.set_cursor_visible(False)
        self.scrollview = gtk.ScrolledWindow()
        self.scrollview.add_with_viewport(self.textview)
        self.scrollview.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        self.frame.add(self.scrollview)
        self.frame.show_all()

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
        self.text_buffer.set_text('')

class GameFacesUI:
    def __init__(self):
        # ivars
        self.player = GFPlayer(-1, False, '')
    
        window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        window.connect("destroy", lambda w: gtk.main_quit())
        window.set_title("Game")
        window.resize(500, 250)

        self.content_vbox = gtk.VBox(False, 8)
        self.content_vbox.set_border_width(10)
        window.add(self.content_vbox)
        
        self.console = GFConsole()
        self.content_vbox.pack_end(self.console.frame)
        
        window.show_all()
        self.register_player()
        
    def register_player(self):
        self.console.append_text("Connecting to Game Master...\n")
        rospy.init_node('game_faces_ui', anonymous=True)
        rospy.Subscriber("game_master", TwoPersonGame, self.play_game)

        rospy.wait_for_service('register_game_player')
        try:
            reg = rospy.ServiceProxy('register_game_player', RegisterGamePlayer)
            response = reg(str(roslib.network.get_host_name()))
            self.player.player_id = response.player_id
        except rospy.ServiceException, e:
            print "Could not get a player_id from master"
            sys.exit(1)
        
        self.console.append_text("Ready to play.\nWaiting for your opponent...\n")

    def play_game(self, gamedata):
        self.console.append_text("\nStarting game: %s\n\n\n" %gamedata.game_type)
        time.sleep(1.0)
        self.player.game_topic_lock.acquire()
        if self.player.game_topic is '':
            is_first_player = (gamedata.first_player == self.player.player_id)
            is_second_player = (gamedata.second_player == self.player.player_id)
            if is_first_player or is_second_player:
                gts = GtkThreadSafe()
                # Register the game playing topic
                game_topic = gamedata.game_topic
                self.player.is_first = is_first_player
                self.player.game_topic = game_topic
                game_type = str(gamedata.game_type)
                if game_type == "TrustGame":
                    #self.the_game = TrustGame(self)
                    with gts:
                        print 'Start TrustGame'
                elif game_type == "Prisoners":
                    #self.the_game = Prisoners(self)
                    with gts:
                        print 'Start Prisoners'
                elif game_type == "Ultimatum":
                    with gts:
                        self.the_game = UltimatumGame(self.console, self.player)
                        self.content_vbox.pack_start(self.the_game.frame, expand=False)

                else:
                    with gts:
                        print "Didnt match game_type: %s" %game_type
                self.the_game.take_first_turn()
        else:
            print "skipped a game message from master, because I'm already in a game"
        self.player.game_topic_lock.release()

if __name__ == "__main__":
    g = GameFacesUI()
    gtk.main()
