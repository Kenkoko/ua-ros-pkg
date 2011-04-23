import roslib
import rospy
from game_faces.msg import GamePlay
from Game import Game

from video_wrapper import videowrapper

import sys
import time
import socket
import threading

import pygtk
pygtk.require('2.0')
import gtk

TIME_BETWEEN_GAMES = 2 #seconds


# NOTE on video_wrapper - the play_numbers are all shifted down by one:
#  0 - game start (player 2)
#  1 - player 1 sends an offer
#  2 - player 2 receives offer
#  3 - player 2 accepts or rejects
#  4 - player 1 receives accept or reject


class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class UltimatumGameController:
    def __init__(self, parent, shared_console, player, log, video = None):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.log = log
        self.video = video

        self.isgame = True
        self.payoff = 0

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(not self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        self.title = "Ultimatum"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
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
        
        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=10, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)

        self.view.show_all()
        self.toggle_user_interaction()

  
# This is one: def make offer: FIRST CHANGE


    def ok_button_clicked(self, widget, data=None):
        timenow = time.time()
        
        offer = self.spin_bid.get_value_as_int()
        
        # Player 1 as they send their offer
        if self.video is not None:
            self.video.send_msg(play_number=1, amount=offer, msgtime=timenow)
        
        self.set_balance(10 - offer)
        gp = GamePlay(play_number=1,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
        self.toggle_user_interaction()
        self.shared_console.append_text('\n\nYou sent %d.\nWaiting for second player...\n' %offer)
        self.log.log("Player sent %d" %offer)
    
    def set_balance(self, balance):
        self.label_balance_amt.set_markup('<span size="20000" weight="bold">%d</span>' %balance)
        self.payoff = balance
    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)
    
    def take_first_turn(self):
        
# NOTE: CAN AVOID CRASH BY SKIPPINGH THIS IF SENTENCE: THE LOGIC THOUGH MAY MAKE PROBLEM WITH MULTIPLE PLAYERS AND WE NEED TO WORK MORE ON THIS POINT

        if not self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
        time.sleep(0.1)     # This is necessary to make sure the video thread has had time to update the game topic by now; otherwise it will publish on old topic
        if self.video is not None:      # This will send duplicate message for Player 1 and 2, but master will keep the most recent
            self.video.send_msg(play_number=0,amount=-1, msgtime = time.time())
        
        
    def take_turn(self, game_play):
        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            self.log.log("Starting Ultimatum game")
            if self.player.is_first:
                with gts:
                    self.set_balance(10)
                    self.toggle_user_interaction()
                    self.shared_console.append_text("Choose an amount, between 0 and 10 and press OK\n")
            else:
                with gts:
                    self.shared_console.append_text("Waiting for first player...\n")
        elif play_number == 1:
            if self.player.is_first:
                pass   
            else:
                with gts:
                    self.log.log("Player received offer of %d" %game_play.amount)
                    label_prompt = gtk.Label('Player 1 has sent %d.\nYou can either accept or reject this offer.' %game_play.amount)
                    dialog = gtk.Dialog('Choose One', self.parent, gtk.DIALOG_MODAL,
                                    ('Reject', gtk.RESPONSE_REJECT,
                                      'Accept', gtk.RESPONSE_ACCEPT)) 
                    dialog.vbox.pack_start(label_prompt, True, True, 0)
                    label_prompt.show()

                    # Reaction as Player 2 receives Player 1's offer
                    if self.video is not None:
                        self.video.send_msg(play_number=2,amount=game_play.amount, msgtime = time.time())

                    dialog.connect("delete_event", lambda w, e : None)
                    response = gtk.RESPONSE_DELETE_EVENT
                    while response == gtk.RESPONSE_DELETE_EVENT:
                        response = dialog.run()
                    dialog.destroy()
                    if response == gtk.RESPONSE_REJECT:
                        new_balance = 0
                    elif response == gtk.RESPONSE_ACCEPT:
                        new_balance = game_play.amount
                    
                    # Player 2 as they accept or reject
                    if self.video is not None:
                        self.video.send_msg(play_number=3,amount=new_balance, msgtime = time.time())

                    if response == gtk.RESPONSE_REJECT:
                        self.log.log("Player rejected offer of %d" %game_play.amount)
                    elif response == gtk.RESPONSE_ACCEPT:
                        self.log.log("Player accepted offer of %d" %game_play.amount)

                    self.set_balance(new_balance)
                    self.shared_console.append_text('\n\nYour payoff is now: %d' %new_balance)
                    
                    gp = GamePlay(play_number=2,amount=new_balance, player_id=self.player.player_id)
                    gp.header.stamp = rospy.Time.now()
                    self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text('\n\nYour offer was ')
                    if game_play.amount == 0:
                        self.shared_console.append_text('rejected. Your payoff is now 0.\n')
                        self.set_balance(0)
                        self.log.log("Player 2 rejected the offer")
                    else:
                        new_balance = 10 - game_play.amount
                        self.shared_console.append_text('accepted. Your payoff is now %d.\n' %new_balance)
                        self.set_balance(new_balance)
                        self.log.log("Player 2 accepted the offer")
                        
                    # Reaction upon receiving player 2's acceptance or rejection
                    if self.video is not None:
                        self.video.send_msg(play_number=4,amount=-2, msgtime = time.time())

                gp = GamePlay(play_number=3,amount=-2, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                time.sleep(TIME_BETWEEN_GAMES)
                self.play_pub.publish(gp)
            else:
                pass
            with gts:
                self.shared_console.append_text("\n\nPlease wait for the next game to start.\n")
                self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()
        
