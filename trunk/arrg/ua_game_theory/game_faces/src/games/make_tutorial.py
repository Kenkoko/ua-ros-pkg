import roslib
import rospy
from game_faces.msg import GamePlay

import sys
import time
import socket
import threading

import random

import pygtk
pygtk.require('2.0')
import gtk

import textwrap
from Tkinter import *

from logger import logger

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class GeneralInstructions:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0

        self.log = log
        
        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)

        self.title = "Instruction"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)

        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)

        self.view.show_all()
        self.toggle_user_interaction()
        

    def ok_button_clicked(self, widget, data=None):
        self.play_number +=1
        self.toggle_user_interaction()
        gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)

    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self,game_play):

        rs_1 = "Thank you for coming.  The instructions you will read are meant to be self-explanatory.  If, after you have read through the instructions, you still have questions, please raise your hand and someone will come by to help you.  Now that the study has begun, we ask that you do not talk, at all, during the experiment. "

        rs_2 = "You will complete three experiments.  We will explain the detailed rules for each experiment at the beginning of the experiment.  "

        rs_3 = "If you follow the instructions carefully and make good decisions, you can earn a notable amount of money.  Two rounds from each experiment will be randomly selected for payment, and you will receive 10%, in real money, of the experimental money you earned on those rounds.  You will be paid in private and in cash at the end of the study."

        s_1=textwrap.fill(rs_1,70)
        s_2=textwrap.fill(rs_2,70)
        s_3=textwrap.fill(rs_3,70)

        gts = GtkThreadSafe()
        if self.play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.toggle_user_interaction()
        elif self.play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                self.play_number = 2
                gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif self.play_number == 2:
            if self.player.is_first:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()


    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()

class Demographic:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0
        
        self.log = log

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)

        self.title = ""
        self.view = gtk.Frame(self.title)
        question_box = gtk.VBox(False, 8)
        self.view.add(question_box)
        
        # Biological Sex
        self.question_one = gtk.HBox(False,8)
        question_box.pack_start(self.question_one, expand=False, fill=False)
        label = gtk.Label("Biological sex:")
        self.question_one.pack_start(label, False, False, 0)
        label.show()

        self.malebutton = gtk.RadioButton(None, "male")
        self.malebutton.connect("toggled", self.rb_callback, "male")
        self.question_one.add(self.malebutton)
        self.femalebutton = gtk.RadioButton(self.malebutton, "female")
        self.femalebutton.connect("toggled", self.rb_callback, "male")
        self.question_one.add(self.femalebutton)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()
        
        # Date of Birth
        self.question_two = gtk.HBox(False,8)
        question_box.pack_start(self.question_two, expand=False, fill=False)
        label = gtk.Label("Date of birth:")
        self.question_two.pack_start(label, False, False, 0)
        label.show()

        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, True, True, 5)

        label = gtk.Label("Day :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, True, 0)
  
        adj = gtk.Adjustment(1.0, 1.0, 31.0, 1.0, 5.0, 0.0)
        self.dayspinner = gtk.SpinButton(adj, 0, 0)
        self.dayspinner.set_wrap(True)
        b_box.pack_start(self.dayspinner, False, True, 0)
        
        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, True, True, 5)
        
        label = gtk.Label("Month :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, True, 0)

        adj = gtk.Adjustment(1.0, 1.0, 12.0, 1.0, 5.0, 0.0)
        self.monthspinner = gtk.SpinButton(adj, 0, 0)
        self.monthspinner.set_wrap(True)
        b_box.pack_start(self.monthspinner, False, True, 0)
  
        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, True, True, 5)
        
        label = gtk.Label("Year :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, True, 0)
  
        adj = gtk.Adjustment(1980.0, 1900.0, 2000.0, 1.0, 100.0, 0.0)
        self.yearspinner = gtk.SpinButton(adj, 0, 0)
        self.yearspinner.set_wrap(False)
        self.yearspinner.set_size_request(55, -1)
        b_box.pack_start(self.yearspinner, False, True, 0)

        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()
        
        # Ethnicity
        self.question_three = gtk.HBox(False,8)
        question_box.pack_start(self.question_three, expand=False, fill=False)
        
        self.ethcombobox = gtk.combo_box_new_text()
        self.question_three.add(self.ethcombobox)
        self.ethcombobox.append_text('Ethnicity:')
        self.ethcombobox.append_text('American Indian/Alaskan Native')
        self.ethcombobox.append_text('Asian/Pacific Islander')
        self.ethcombobox.append_text('Hispanic')
        self.ethcombobox.append_text('White non-Hispanic')
        self.ethcombobox.append_text('Black non-Hispanic')
        self.ethcombobox.append_text('Multiracial')
        self.ethcombobox.append_text('Other')
        self.ethcombobox.connect('changed', self.changed_cb)
        self.ethcombobox.set_active(0)

        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # How many GT
        
        self.question_four = gtk.HBox(False,8)
        question_box.pack_start(self.question_four, expand=False, fill=False)
        label = gtk.Label("How many courses have you taken that discussed game theory?")
        self.question_four.pack_start(label, False, False, 0)
        label.show()
        
        self.gt_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=20, step_incr=1))
        self.question_four.add(self.gt_spin_question)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # How many Math
        
        self.question_five = gtk.HBox(False,8)
        question_box.pack_start(self.question_five, expand=False, fill=False)
        label = gtk.Label("How many mathematics and economics classes have you taken?")
        self.question_five.pack_start(label, False, False, 0)
        label.show()
        
        self.math_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=20, step_incr=1))
        self.question_five.add(self.math_spin_question)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # Go on
        self.question_six = gtk.HBox(False,8)
        question_box.pack_start(self.question_six, expand=False, fill=False)
        label = gtk.Label("I am ready to start the experiment")
        self.question_six.pack_start(label, False, False, 0)
        label.show()
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.question_six.add(self.button_bid)
        
        self.view.show_all()
        self.toggle_user_interaction()
        
    def rb_callback(self, widget, data=None):
        print "%s was toggled %s"

    def changed_cb(self, combobox):
        model = combobox.get_model()
        index = combobox.get_active()
        if index:
            print model[index][0]
        return
        
    def ok_button_clicked(self, widget, data=None):
        self.play_number +=1
        self.log_answers()
        self.toggle_user_interaction()
        gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)

    def log_answers(self):
        # sex, dob, ethnicity, game theory classes, math classes
        demo = "Sex: " + ( "Male" if self.malebutton.get_active() else "Female") + "\n" + \
               "DOB: " + str(self.yearspinner.get_value_as_int()) + "-" + str(self.monthspinner.get_value_as_int()) + "-" + str(self.dayspinner.get_value_as_int()) + "\n" + \
               "Ethnicity: " + str(self.ethcombobox.get_model()[self.ethcombobox.get_active()][0]) + "\n" + \
               "Game theory classes: " + str(self.gt_spin_question.get_value_as_int()) + "\n" + \
               "Math classes: " + str(self.math_spin_question.get_value_as_int()) + "\n"

        self.log.log("Answers to demographic questions:\n" + demo)

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.question_one.set_sensitive(self.enabled)
        self.question_two.set_sensitive(self.enabled)
        self.question_three.set_sensitive(self.enabled)
        self.question_four.set_sensitive(self.enabled)
        self.question_five.set_sensitive(self.enabled)
        self.question_six.set_sensitive(self.enabled)
        
    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
    def take_turn(self,game_play):
        rs_1 = "Please answer to the questions above and then press 'OK'."
        s_1=textwrap.fill(rs_1,70)
        gts = GtkThreadSafe()
        if self.play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.toggle_user_interaction()
        elif self.play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                self.play_number = 2
                gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif self.play_number == 2:
            if self.player.is_first:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()


    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()


class UltimatumTutorial:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0
        
        self.log = log

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        self.title = "Ultimatum Game: Tutorial"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=10, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)

        self.view.show_all()
        self.toggle_user_interaction()


    def ok_button_clicked(self, widget, data=None):
        offer = self.spin_bid.get_value_as_int()
        self.log.log("Ultimatum expectation is %d" %offer)
        self.play_number +=1
        self.toggle_user_interaction()
        gp = GamePlay(play_number=self.play_number,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)
    
    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):


        rs_1 = "For each round of this experiment, you will be paired with a computer player that has been programmed to play the game according to data collected from subjects whom had previously played the game. Your decision will also be used to program the computer for future experimental sessions and therefore will determine other subjects' payoffs."
        
        rs_2 = "You will be designated as either Player 1 or Player 2.  A sum of $10 has been provisionally allocated to Player 1, who will propose how much of this each player is to receive.  To do this, Player 1 will indicate how much money to give to Player 2.  He or she may choose any amount between zero and $10.  The amount that Player 1 is to receive is simply the amount to be divided, $10, minus the amount sent to Player 2."
        
        rs_3 = "Once Player 1 has made a proposal, Player 2 will be given the chance to accept or reject the proposal.  If Player 2 accepts the proposal, then the amount of money will be divided as specified by Player 1.  If Player 2 rejects the proposal, both Player 1 and Player 2's payoff will be zero.  After Player 2's decision, the interaction is over."
        
        rs_4 = "This experiment consists of eighteen rounds.  You will be paired with a different person for each round.  You will never interact with the same person in the same role twice.   "
        
        rs_5 = "This experiment consists of eighteen rounds. Remember that two rounds will be randomly selected for real payment, and you will receive 10%, in real money, of the experimental money you earned on those round."
        
        rs_6 = "By using the control above, please tell us what you expect to receive on average when you are the second player in the game. Remember that the first player can give you between $0 and $10, therefore select a number between 0 and 10. Once you made your estimation, please press the OK button."
        
        s_1=textwrap.fill(rs_1,65)
        s_2=textwrap.fill(rs_2,65)
        s_3=textwrap.fill(rs_3,65)
        s_4=textwrap.fill(rs_4,65)
        s_5=textwrap.fill(rs_5,65)
        s_6=textwrap.fill(rs_6,65)

        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_4)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_5)
                    self.toggle_user_interaction()
        elif play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                with gts:
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_6)
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()

class TrustTutorial:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0
        
        self.log = log
        
        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.title = "Trust Game: Tutorial"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)
        
        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)
        
        self.view.show_all()
        self.toggle_user_interaction()
    
    def ok_button_clicked(self, widget, data=None):
        offer = self.spin_bid.get_value_as_int()
        self.log.log("Trust game expectations are %d" %offer)
        self.play_number +=1
        self.toggle_user_interaction()
        gp = GamePlay(play_number=self.play_number,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
        if self.play_number == 3:
            self.unregister_game()

    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)

    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):

        rs_1 = "For each round of this experiment, you will be paired with a computer player that has been programmed to play the game according to data collected from subjects whom had previously played the game. Your decision will also be used to program the computer for future experimental sessions and therefore will determine other subjects' payoffs."
        
        rs_2 = "You will be designated as either Player 1 or Player 2.  A sum of $10 has been provisionally allocated to Player 1 who can then decide to transfer any amount of this to Player 2.  Each dollar Player 1 sends will be tripled, and the product will be delivered to Player 2. For example, if Player 1 sent $3, then 3*3 = $9 would be delivered; if Player 1 had instead sent $7, then 7*3 = $21 would be delivered."
        
        rs_3 = "Once Player 1 has made a transfer, Player 2 will be given the chance to send any amount of the transferred money back to Player 1. Player 2 can choose any number between zero and the tripled amount he or she received.  For example, if Player 1 sent $3, then Player 2 could send back any amount between zero and $9; if Player 1 had sent $7, then Player 2 could send back any amount between zero and $21.  After Player 2's decision, the interaction is over."
        
        rs_4 = "The final payoff for Player 1 is $10, minus however much he or she transferred to Player 2, plus however much Player 2 sent back.  The final payoff for Player 2 is the (tripled) amount transferred to him or her by Player 1, minus however much he or she sends back to Player 1. For example, if Player 1 sent $3, and Player 2 sent back $2, then Player 1 would end up with $10 - $3 + $2 = $9 and Player 2 would end up with $9 - $2 = $7; if Player 1 had sent $7, and Player 2 sent back $9, then Player 1 would end up with $10 - $7 + $9 = $12 and Player 2 would end up with $21 - $9 = $12."
        
        rs_5 = "This experiment consists of eighteen rounds."
        
        rs_6 = "Remember that two rounds will be randomly selected for real payment, and you will receive 10%, in real money, of the experimental money you earned on those rounds."
        
        rs_7 = "By using the control above, please tell us what you expect to receive on average when you are the first player in the game. Remember that this depends on the ammount that you are going to send to the second player, therfore, rather than in terms of dollar, please express you estimation in percentage - that is choose a number between 0% and 100%. Once you made your estimation, please press the OK button."
        
        rs_8 = "By using the control above, please tell us what you expect to receive on average when you are the second player in the game. Remember that the first player can give you between $0 and $10, therefore choose a number between 0 and 10. Once you made your estimation, please press the OK button."

        s_1=textwrap.fill(rs_1,65)
        s_2=textwrap.fill(rs_2,65)
        s_3=textwrap.fill(rs_3,65)
        s_4=textwrap.fill(rs_4,65)
        s_5=textwrap.fill(rs_5,65)
        s_6=textwrap.fill(rs_6,65)
        s_7=textwrap.fill(rs_7,65)
        s_8=textwrap.fill(rs_8,65)

        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_4)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_5)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_6)
                    self.shared_console.append_text("\n\n")
                    self.toggle_user_interaction()
        elif play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                with gts:
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_7)
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                self.shared_console.clear()
                with gts:
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_8)
                    self.toggle_user_interaction()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()


class PrisonersTutorial:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0
        self.row_choice = 4
        self.col_choice = 4
        
        self.log = log

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
        self.title = "Prisoners"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)
        
        self.button_top = None
        self.button_bottom = None
        self.button_left = None
        self.button_right = None

        nrows = 3
        ncols = 3

        self.button_top = gtk.CheckButton("Top")
        self.button_top.connect("toggled", self.checkbox_toggled, "Top")
        self.button_bottom = gtk.CheckButton("Bottom")
        self.button_bottom.connect("toggled", self.checkbox_toggled, "Bottom")
        self.button_left = gtk.CheckButton("Left")
        self.button_left.connect("toggled", self.checkbox_toggled, "Left")
        self.button_right = gtk.CheckButton("Right")
        self.button_right.connect("toggled", self.checkbox_toggled, "Right")
        self.table = gtk.Table(rows=nrows, columns=ncols, homogeneous=False)
        self.label_tl = gtk.Label("10 | 10")
        self.label_tr = gtk.Label("0 | 12")
        self.label_bl = gtk.Label("12 | 0")
        self.label_br = gtk.Label("2 | 2")

        self.table.attach(self.button_left, 1, 2, 0, 1, xoptions=gtk.EXPAND)
        self.table.attach(self.button_right, 2, 3, 0, 1, xoptions=gtk.EXPAND)
        self.table.attach(self.button_top, 0, 1, 1, 2, xoptions=gtk.EXPAND)
        self.table.attach(self.button_bottom, 0, 1, 2, 3, xoptions=gtk.EXPAND)
        self.table.attach(self.label_tl, 1, 2, 1, 2, xoptions=gtk.EXPAND)
        self.table.attach(self.label_bl, 1, 2, 2, 3, xoptions=gtk.EXPAND)
        self.table.attach(self.label_tr, 2, 3, 1, 2, xoptions=gtk.EXPAND)
        self.table.attach(self.label_br, 2, 3, 2, 3, xoptions=gtk.EXPAND)
        self.box_control.add(self.table)

        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)

        self.view.show_all()
        self.toggle_user_interaction()

    def checkbox_toggled(self, widget, data=None):
        if widget == self.button_left:
            self.col_choice = 0 if widget.get_active() else 4
            self.button_right.set_sensitive(not widget.get_active())
        elif widget == self.button_right:
            self.col_choice = 1 if widget.get_active() else 4
            self.button_left.set_sensitive(not widget.get_active())
        elif widget == self.button_top:
            self.row_choice = 2 if widget.get_active() else 4
            self.button_bottom.set_sensitive(not widget.get_active())
        elif widget == self.button_bottom:
            self.row_choice = 3 if widget.get_active() else 4
            self.button_top.set_sensitive(not widget.get_active())

    def ok_button_clicked(self, widget, data=None):
        self.toggle_user_interaction()
        self.play_number +=1
        if self.player.is_first:
            if self.play_number ==1:
                gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            else:
                amt_val_1 = self.row_choice
                amt_val_2 = self.col_choice
                self.log.log("Row expected is %d" %amt_val_1)
                self.log.log("Column expected is %d" %amt_val_2)
                gp = GamePlay(play_number=self.play_number,amount=10*amt_val_1+amt_val_2, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
    
    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)
    
    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):

        rs_1 = "For each round of this experiment, you will be paired with a computer player that has been programmed to play the game according to data collected from subjects whom had previously played the game. Your decision will also be used to program the computer for future experimental sessions and therefore will determine other subjects' payoffs."
        
        rs_2 = "You will be designated as either Player 1 or Player 2. Player 1 will be able to choose either the TOP or the BOTTOM row of the table provided. Simultaneously, Player 2 will be able to choose either the LEFT or the RIGHT column of the same table. The numbers in each cell of the table represent payoff amounts. In each cell the number on the left is the possible payoff for Player 1 and the number on the right is the possible payoff for Player 2. "
        
        rs_3 = "For example, if Player 1 chooses the TOP row and Player 2 chooses the LEFT column then both players will receive 10 units of payoff. Likewise, if Player 2 choose LEFT but Player 1 chooses the BOTTOM row, Player 1 will get nothing and Player 2 will get $12."
        
        rs_4 = "This experiment consists of nine rounds. "
        
        rs_5 = "Remember that two rounds will be randomly selected for real payment, and you will receive 10%, in real money, of the experimental money you earned on those rounds."
        
        rs_6 = "By using the check boxes above, please tell us what you expect to be the most frequently visited cell. Once you made your estimation, please press the OK button."
        
        s_1=textwrap.fill(rs_1,65)
        s_2=textwrap.fill(rs_2,65)
        s_3=textwrap.fill(rs_3,65)
        s_4=textwrap.fill(rs_4,65)
        s_5=textwrap.fill(rs_5,65)
        s_6=textwrap.fill(rs_6,65)

        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_4)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_5)
                    self.shared_console.append_text("\n\n")
                    self.toggle_user_interaction()
        elif play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                with gts:
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_6)
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()

class ArmBanditTutorial:
    def __init__(self, parent, shared_console, player, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0
        self.is_choosing = 0
#        self.lottery = self.lottery_image(self.parent, self.play_number, 5)

        self.log = log

        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        self.title = "Arm-Bandit Task: Tutorial"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        
        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)

        pixbufanim = gtk.gdk.PixbufAnimation("-2.jpg")
        image = gtk.Image()
        image.set_from_animation(pixbufanim)
        image.set_size_request(100,150)
        image.show()
        self.button_bid = gtk.Button()
        self.button_bid.add(image)
        self.box_control.add(self.button_bid)
        self.button_bid.connect("clicked", self.ok_button_clicked, 1)
        self.spin_bid_1 = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.box_control.add(self.spin_bid_1)

        image = gtk.Image()
        image.set_from_file("-3.jpg")
        image.set_size_request(100,150)
        image.show()
        self.button_bid = gtk.Button()
        self.button_bid.add(image)
        self.box_control.add(self.button_bid)
        self.button_bid.connect("clicked", self.ok_button_clicked, 2)
        self.spin_bid_2 = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.box_control.add(self.spin_bid_2)

        image = gtk.Image()
        image.set_from_file("-4.jpg")
        image.set_size_request(100,150)
        image.show()
        self.button_bid = gtk.Button()
        self.button_bid.add(image)
        self.box_control.add(self.button_bid)
        self.button_bid.connect("clicked", self.ok_button_clicked, 3)
        self.spin_bid_3 = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.box_control.add(self.spin_bid_3)

        self.view.show_all()
        self.toggle_user_interaction()

    def ok_button_clicked(self, widget, data=None):
        exp_1 = self.spin_bid_1.get_value_as_int()
        exp_2 = self.spin_bid_2.get_value_as_int()
        exp_3 = self.spin_bid_3.get_value_as_int()
        self.log.log("Expectation for Arm One is %d" %exp_1)
        self.log.log("Expectation for Arm Two is %d" %exp_2)
        self.log.log("Expectation for Arm Three is %d" %exp_3)
        self.toggle_user_interaction()
        self.play_number +=1
        if self.player.is_first:
            gp = GamePlay(play_number=self.play_number,amount=1000000*exp_1+1000*exp_2+exp_3, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
#        offer = self.spin_bid.get_value_as_int()
#        self.play_number +=1
#        self.toggle_user_interaction()
#        gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
#       gp.header.stamp = rospy.Time.now()
#        self.play_pub.publish(gp)

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)
    
    def take_first_turn(self):
        if self.player.is_first:
            gp = GamePlay(play_number=0,amount=-1, player_id=self.player.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):

        rs_1 = "For each round of this experiment, you will be able to choose between three different slot machines: slot-machine 1, 2, and 3. You can choose which slot machine to bet on and your payoff for the round will be the amount outputted by the chosen slot machine."
        
        rs_2 = "Each slot machine provides a reward between 1 and 100. Your objective for this game is to win as much as possible."

        rs_3 = "This experiment consists of 125 rounds. "
        
        rs_4 = "This game will be paid in accordance with the following procedure. We will sum the payoff of each play and then divide it by the number of rounds played (i.e. 125) and the median payoff (which is 50)."
        
        rs_5 = "For example, suppose that you always received 100 from your plays, your overall earning in payoff units will be 12,500. The conversion in Dollars is: Total payoff/(number of rounds * median payoff) = 12500/(125*50) = $2."
        
        s_1=textwrap.fill(rs_1,70)
        s_2=textwrap.fill(rs_2,70)
        s_3=textwrap.fill(rs_3,70)
        s_4=textwrap.fill(rs_4,70)
        s_5=textwrap.fill(rs_5,70)

        gts = GtkThreadSafe()
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_4)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_5)
                    self.shared_console.append_text("\n\n")
                    self.toggle_user_interaction()
        elif play_number == 1:
            if self.player.is_first:
                self.shared_console.clear()
                with gts:
                    self.toggle_user_interaction()
                    self.shared_console.append_text("\nBy using the control above, please tell us how much you expect\n to receive from each lottery\n\n We know that you do not have enough information, but try to guess anyway. \n\nIndicate the amount in the three box and then press OK")
        elif play_number == 2:
            if self.player.is_first:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()
    
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()



class PaymentScreen:
    def __init__(self, parent, shared_console, player, results, log):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.play_number = 0

        self.log = log
        
        self.results = results
        
        self.enabled = True
#        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(self.player.is_first))
#        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)

        self.title = "Payment"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)

        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)
        
        self.label_balance_desc = gtk.Label('Your total payment is')
        self.box_status.add(self.label_balance_desc)
        
        self.label_balance_amt = gtk.Label()
        self.set_balance(0)
        self.box_status.add(self.label_balance_amt)

        self.box_control = gtk.HBox(False, 8)        
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        # No ok button
        #self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        #self.button_bid.set_size_request(80, 30)
        #self.button_bid.connect("clicked", self.ok_button_clicked)
        #self.box_control.add(self.button_bid)

        self.view.show_all()
        #self.toggle_user_interaction()

    def set_balance(self, balance):
        self.label_balance_amt.set_markup('<span size="20000" weight="bold">%d</span>' %balance)
        self.payoff = balance
        

#    def ok_button_clicked(self, widget, data=None):
#        self.play_number +=1
#        self.toggle_user_interaction()
#        gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
#        gp.header.stamp = rospy.Time.now()
#        self.play_pub.publish(gp)
#        self.log.log("Ending Payment module")

#    def toggle_user_interaction(self):
#        self.enabled = not self.enabled
#        self.box_control.set_sensitive(self.enabled)

    def take_first_turn(self):
        self.log.log("Starting Payment module")
        #if self.player.is_first:
        #    gp = GamePlay(play_number=0,amount=0, player_id=self.player.player_id)
        #    gp.header.stamp = rospy.Time.now()
        #    self.play_pub.publish(gp)

    def take_turn(self,game_play):

        # Figure out payment amount: select 2 rounds from each experiment
        print "original results: ", self.results
        random.shuffle( self.results )
        print "shuffled results: ", self.results
        
        res = {}
        for result in self.results:
            if result[0] in res:
                if (len(res[result[0]]) < 2):
                    res[result[0]].append(result[1])
            else:
                res[result[0]] = [result[1]]
        print "dictionary: ", res
        
        ressum = {}
        total = 0
        for game_name in res.keys():
            ressum[game_name] = sum(res[game_name])
            total += ressum[game_name]	# Here, change payment values for various games ("if game_name == 'one armed'...)
        
        print ressum
        
        rs_1 = "Your payment has been calculated by picking two rounds randomly from each game.\n"
        rs_2 = "Here are the games and the associated payoffs: " + str(res) + "\n"
        rs_3 = "Your total payoff is: $" + str(total) + "\n"
        rs_4 = "Please wait for the researcher to award payment\n"
        
        s_1=textwrap.fill(rs_1,70)
        s_2=textwrap.fill(rs_2,70)
        s_3=textwrap.fill(rs_3,70)
        s_4=textwrap.fill(rs_4,70)

        gts = GtkThreadSafe()
        if self.play_number == 0:
            if self.player.is_first:
                self.set_balance(total)
                self.log.log("Monetasry payof is %d" %total)
                with gts:
                    self.shared_console.append_text(s_1)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_2)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_3)
                    self.shared_console.append_text("\n\n")
                    self.shared_console.append_text(s_4)
                    self.shared_console.append_text("\n\n")
#                    self.toggle_user_interaction()
#        elif self.play_number == 1:
#            if self.player.is_first:
#                self.shared_console.clear()
#                self.play_number = 2
#                gp = GamePlay(play_number=self.play_number,amount=-1, player_id=self.player.player_id)
#                gp.header.stamp = rospy.Time.now()
#                self.play_pub.publish(gp)
#        elif self.play_number == 2:
#            if self.player.is_first:
#                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
#                gp.header.stamp = rospy.Time.now()
#                self.play_pub.publish(gp)
#            self.unregister_game()


    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_pub.unregister()
        self.play_sub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()


