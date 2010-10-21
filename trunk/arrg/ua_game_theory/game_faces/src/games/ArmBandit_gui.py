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
#  0 - game starts
#  1 - player chooses slot machine
#  2 - player waiting for machine
#  3 - player sees outcome


TIME_BETWEEN_GAMES = 3 #seconds

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class ArmBanditController:
    def __init__(self, parent, shared_console, player, lottery, log, video = None):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.log = log
        self.play_number = 0
        self.balance = 0
        self.lottery = lottery
        print lottery
        
        self.video = video

        self.isgame = True
        self.payoff = 0

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.title = "Arm-Bandit Task"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)
        
        self.label_balance_desc = gtk.Label('Your balance for this round is:')
        self.box_status.add(self.label_balance_desc)
        
        self.label_balance_amt = gtk.Label()
        self.set_balance(0)
        self.box_status.add(self.label_balance_amt)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        pixbufanim = gtk.gdk.PixbufAnimation("-2.jpg")
        image = gtk.Image()
        image.set_from_animation(pixbufanim)
        image.set_size_request(200,150)
        image.show()
        self.button_bid = gtk.Button()
        self.button_bid.add(image)
        self.button_bid.connect("clicked", self.ok_button_clicked, 0)
        self.box_control.add(self.button_bid)

        image = gtk.Image()
        image.set_from_file("-3.jpg")
        image.set_size_request(200,150)
        image.show()
        self.button_bid= gtk.Button()
        self.button_bid.add(image)
        self.button_bid.connect("clicked", self.ok_button_clicked, 1)
        self.box_control.add(self.button_bid)

        image = gtk.Image()
        image.set_from_file("-4.jpg")
        image.set_size_request(200,150)
        image.show()
        self.button_bid = gtk.Button()
        self.button_bid.add(image)
        self.button_bid.connect("clicked", self.ok_button_clicked, 2)
        self.box_control.add(self.button_bid)

        self.view.show_all()
        self.toggle_user_interaction()

    def set_balance(self, balance):
        self.label_balance_amt.set_markup('<span size="20000" weight="bold">%d</span>' %balance)
        self.payoff = balance

    def ok_button_clicked(self, widget, data=None):
        if self.player.is_first:
            self.play_number +=1

            # Player chooses a machine
            if self.video is not None:
                self.video.send_msg(play_number=self.play_number, amount=data, msgtime = time.time())

            gp = GamePlay(play_number=self.play_number,amount=data, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)

    def take_first_turn(self):
        if self.player.is_first:
            # Game starts
            if self.video is not None:
                self.video.send_msg(play_number=0, amount=-1, msgtime = time.time())

            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if self.play_number == 0:
            self.log.log("Starting Arm Bandit Task")
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text("\nChose the lottery you want to play with for this round.")
                    self.toggle_user_interaction()
        elif play_number == 1:
            if self.player.is_first:
                with gts:
                    if game_play.amount == 0:
                        self.shared_console.append_text("\n\nYou chosed to play with the FIRST slot machine. \n\nThe lottery is currently running!!")
                    elif game_play.amount == 1:
                        self.shared_console.append_text("\n\nYou chosed to play with the SECOND slot machine. \n\nThe lottery is currently running!!")
                    else:
                        self.shared_console.append_text("\n\nYou chosed to play with the THIRD slot machine. \n\nThe lottery is currently running!!")
                        
                    self.balance = round(self.lottery[game_play.amount])

                    # Player waiting for machine
                    if self.video is not None:
                        self.video.send_msg(play_number=2, amount=game_play.amount, msgtime = time.time())

                self.log.log("Player chose lottery %d" %game_play.amount)
                self.log.log("The lottery paid %d" %self.balance)
                gp = GamePlay(play_number=2,amount=game_play.amount, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                time.sleep(1.5)
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first:
                self.set_balance(self.balance)
                with gts:
                    self.shared_console.append_text('\n\nYour winning from the lottery are %d.\n' %self.balance)
                    # Player receives payoff
                    if self.video is not None:
                        self.video.send_msg(play_number=3,amount=self.balance, msgtime = time.time())

                gp = GamePlay(play_number=3,amount=self.balance, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                time.sleep(1.5)
                self.play_pub.publish(gp)
            self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()
