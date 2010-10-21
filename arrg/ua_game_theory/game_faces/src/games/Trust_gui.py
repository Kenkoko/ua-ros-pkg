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


# NOTE on video_wrapper - the play_numbers are as follows:
#  0 - player 1 sends amount
#  1 - player 2 receives offer
#  2 - player 2 sends amount back
#  3 - player 1 receives returned amount


class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class TrustGameController:
    def __init__(self, parent, shared_console, player, log, video = None):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.balance = 0
        self.log = log
        self.video = video
        
        self.isgame = True
        self.payoff = 0

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(not self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.title = "Trust"
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
    
    def ok_button_clicked(self, widget, data=None):
        timenow = time.time()
        
        offer = self.spin_bid.get_value_as_int()
        
        self.toggle_user_interaction()
        if self.player.is_first:
            # Player 1 as they send their amount
            if self.video is not None:
                self.video.send_msg(play_number=0, amount=offer, msgtime=timenow)

            play_num = 1
            self.balance = 10 - offer
            self.set_balance(self.balance)
            self.shared_console.append_text('\n\nYou sent %d.\nYour current payoff is %d\nWaiting for second player...\n' %(offer, self.balance))
            self.log.log("Player sent %d" %offer)

        else:
            # Player 2 as they send their amount back
            if self.video is not None:
                self.video.send_msg(play_number=2, amount=offer, msgtime=timenow)

            play_num = 2
            self.set_balance(self.valueX3 - offer)
            self.shared_console.append_text('\n\nYou returned %d.\nYour current payoff is %d\n' %(offer, self.valueX3 - offer))
            self.log.log("Player returned %d" %offer)
        
        gp = GamePlay(play_number=play_num,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
    
    def set_balance(self, balance):
        self.label_balance_amt.set_markup('<span size="20000" weight="bold">%d</span>' %balance)
        self.payoff = balance
    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)

    def take_first_turn(self):
        if not self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):
        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            self.log.log("Starting Trust Game")
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
                    value = game_play.amount
                    self.valueX3 = value * 3
                    self.log.log("Player received %d" %self.valueX3)
                    s = '\n\nPlayer 1 has sent %d. The tripled value is %d.\nChoose an amount to return to player 1 and press OK.\n' %(value, self.valueX3)
                    self.shared_console.append_text(s)
                    self.spin_bid.get_adjustment().set_lower(0)
                    self.spin_bid.get_adjustment().set_upper(self.valueX3)
                    self.spin_bid.set_value(0)
                    self.set_balance(self.valueX3)
                    self.toggle_user_interaction()
                    
                    # Player 2 as they receive the amount
                    if self.video is not None:
                        self.video.send_msg(play_number=1, amount=self.valueX3, msgtime=time.time())

        elif play_number == 2:
            if self.player.is_first:
                with gts:
                    self.log.log("Player 2 returned %d" %game_play.amount)
                    self.shared_console.append_text('\n\nPlayer 2 returned %d.\n' %game_play.amount)
                    new_balance = self.balance + game_play.amount
                    self.shared_console.append_text('Your payoff is now %d.\n' %new_balance)
                    self.set_balance(new_balance)
                    
                    # Player 1 as they receive the amount back
                    if self.video is not None:
                        self.video.send_msg(play_number=3, amount=game_play.amount, msgtime=time.time())

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

