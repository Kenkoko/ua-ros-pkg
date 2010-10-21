import roslib
import rospy
import numpy
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


# NOTE on video_wrapper - the play_numbers are as follows:
#  0 - player 1 chooses row
#  1 - player 2 chooses column
#  2 - player 1 sees outcome
#  3 - player 2 sees outcome



TIME_BETWEEN_GAMES = 2 #seconds
# There is something to adjust in the delays here
class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class PrisonersGameController:
    def __init__(self, parent, shared_console, player, log, video = None):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.log = log
        
        self.video = video
        
        self.balance = 0
        self.payoffs = (([10,10],[0,12]),([12,0],[2,2]))
        self.row_choice = 2 # choose a number out of range so we know if we dont get a value
        self.col_choice = 2 # choose a number out of range so we know if we dont get a value
        
        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(not self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.isgame = True
        self.payoff = 0

        self.title = "Prisoners"
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
        
        self.button_top = None
        self.button_bottom = None
        self.button_left = None
        self.button_right = None

        if self.player.is_first:
            nrows = 2
            ncols = 3
            
            self.button_top = gtk.CheckButton("Top")
            self.button_top.connect("toggled", self.checkbox_toggled, "Top")
            
            self.button_bottom = gtk.CheckButton("Bottom")
            self.button_bottom.connect("toggled", self.checkbox_toggled, "Bottom")
        else:
            nrows = 3
            ncols = 2
            
            self.button_left = gtk.CheckButton("Left")
            self.button_left.connect("toggled", self.checkbox_toggled, "Left")
            
            self.button_right = gtk.CheckButton("Right")
            self.button_right.connect("toggled", self.checkbox_toggled, "Right")
        
        self.table = gtk.Table(rows=nrows, columns=ncols, homogeneous=False)
        
        self.label_tl = gtk.Label("10 | 10")
        self.label_tr = gtk.Label("0 | 12")
        self.label_bl = gtk.Label("12 | 0")
        self.label_br = gtk.Label("2 | 2")
        
        if self.player.is_first:
            self.table.attach(self.button_top, 0, 1, 0, 1, xoptions=gtk.EXPAND)
            self.table.attach(self.button_bottom, 0, 1, 1, 2, xoptions=gtk.EXPAND)
            self.table.attach(self.label_tl, 1, 2, 0, 1, xoptions=gtk.EXPAND)
            self.table.attach(self.label_bl, 1, 2, 1, 2, xoptions=gtk.EXPAND)
            self.table.attach(self.label_tr, 2, 3, 0, 1, xoptions=gtk.EXPAND)
            self.table.attach(self.label_br, 2, 3, 1, 2, xoptions=gtk.EXPAND)
        else:
            self.table.attach(self.button_left, 0, 1, 0, 1, xoptions=gtk.EXPAND)
            self.table.attach(self.button_right, 1, 2, 0, 1, xoptions=gtk.EXPAND)
            self.table.attach(self.label_tl, 0, 1, 1, 2, xoptions=gtk.EXPAND)
            self.table.attach(self.label_bl, 0, 1, 2, 3, xoptions=gtk.EXPAND)
            self.table.attach(self.label_tr, 1, 2, 1, 2, xoptions=gtk.EXPAND)
            self.table.attach(self.label_br, 1, 2, 2, 3, xoptions=gtk.EXPAND)
        
        self.box_control.add(self.table)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)
        
        self.view.show_all()
        self.toggle_user_interaction()
    
    def checkbox_toggled(self, widget, data=None):
        if widget == self.button_left:
            self.col_choice = 0 if widget.get_active() else 2
            self.button_right.set_sensitive(not widget.get_active())
        elif widget == self.button_right:
            self.col_choice = 1 if widget.get_active() else 2
            self.button_left.set_sensitive(not widget.get_active())
        elif widget == self.button_top:
            self.row_choice = 0 if widget.get_active() else 2
            self.button_bottom.set_sensitive(not widget.get_active())
        elif widget == self.button_bottom:
            self.row_choice = 1 if widget.get_active() else 2
            self.button_top.set_sensitive(not widget.get_active())

    def ok_button_clicked(self, widget, data=None):
        self.toggle_user_interaction()
        if self.player.is_first:
            play_num = 1
            amt_val = self.row_choice
            
            # Player 1 choosing a row
            if self.video is not None:
                self.video.send_msg(play_number=0,amount=amt_val, msgtime = time.time())
        
            self.log.log("Row choice is %d" %amt_val)
#            self.shared_console.append_text('Waiting for the other player...\n')
        else:
            play_num = 2
            amt_val = self.col_choice

            # Player 2 choosing a column
            if self.video is not None:
                self.video.send_msg(play_number=1,amount=amt_val, msgtime = time.time())

            self.log.log("Column choice is %d" %amt_val)
            r_str = ('top', 'bottom')[self.row_choice]
            c_str = ('left', 'right')[self.col_choice]
            new_balance = self.payoffs[self.row_choice][self.col_choice][not self.player.is_first]
# This is to set some delay on the single subjects case, there should be something better
            time.sleep(numpy.random.randint(1,4,size=1))
            self.shared_console.append_text('\n\nYou chose the %s column and player 1 chose the %s row.\n' %(c_str, r_str))
            self.shared_console.append_text('Your payoff is now %d.\n' %new_balance)
            self.set_balance(new_balance)
            
            # Player 2 receiving outcome
            if self.video is not None:
                self.video.send_msg(play_number=3,amount=new_balance, msgtime = time.time())

        
        gp = GamePlay(play_number=play_num,amount=amt_val, player_id=self.player.player_id)
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
# This is definitively not the right way to go, if we had n player simultaneously....haveto figure this out
    def take_turn(self, game_play):
        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            self.log.log("Starting Prisoner Dilemma")
            if self.player.is_first:
                with gts:
                    self.toggle_user_interaction()
                    self.shared_console.append_text("Your possible payoffs are on the left in each cell.\n")
                    self.shared_console.append_text("Please choose the top row or the bottom row and press OK.\n")
            else:
                with gts:
                    self.shared_console.append_text("Your possible payoffs are on the right in each cell.\n")
                    self.shared_console.append_text("Please choose the left column or the right column and press OK.\n")
        elif play_number == 1:
            if self.player.is_first:
                pass
            else:
                with gts:
                    self.row_choice = game_play.amount
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                self.col_choice = game_play.amount
                r_str = ('top', 'bottom')[self.row_choice]
                c_str = ('left', 'right')[self.col_choice]
                with gts:
                    self.col_choice = game_play.amount
                    new_balance = self.payoffs[self.row_choice][self.col_choice][not self.player.is_first]
# this is to set a little delay when playing second, otherwise there is no time between playing and receiving information
                    self.shared_console.append_text('\n\nYou chose the %s row and player 2 chose the %s column.\n' %(r_str, c_str))
                    self.shared_console.append_text('Your payoff is now %d.\n' %new_balance)
                    self.set_balance(new_balance)

                    # Player 1 seeing outcome
                    if self.video is not None:
                        self.video.send_msg(play_number=2,amount=new_balance, msgtime = time.time())

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

