#!/usr/bin/env python


#from game_faces.msg import GamePlay
#from game_faces.msg import TwoPersonGame, GamePlay, GameSummary
import sys
import time
import socket
import threading
import numpy

import random

import pygtk
pygtk.require('2.0')
import gtk

import textwrap
from Tkinter import *

import roslib
roslib.load_manifest('game_faces')
import rospy

import os

# Import functions for starting the video thread
import shlex, subprocess

#from logger import logger
import datetime

from game_faces.srv import *
from game_faces.msg import TwoPersonGame, GamePlay, GameSummary
from games import UltimatumGameController #,UltimatumGameController_computer, UltimatumGameController_mix, Payment
#from games import Welcome

from video_wrapper import videowrapper

#from logger import logger

gtk.gdk.threads_init()

class GtkThreadSafe:
    def __enter__(self):
        gtk.gdk.threads_enter()
    
    def __exit__(self, _type, value, traceback):
        # do any error handling, return False to propogate
        # error, True if handled here
        gtk.gdk.threads_leave()

class Welcome():
    def __init__(self, parent, console, mainvbox, filename, log, condition):
        self.condition = condition
        self.filename = filename
        self.log = log
        self.content_vbox = mainvbox
        self.enabled = True
        self.playing = True
        self.window = parent
        self.console = console
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")

        self.title = "Welcome"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        self.content_vbox.pack_end(self.console.view)

        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.button_bid = gtk.Button('Next')
        self.button_bid.set_size_request(80, 40)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.pack_end(self.button_bid, False, False)
        self.view.show_all()
        self.toggle_user_interaction()
        self.take_turn()

    def step_forward(self):
        self.content_vbox.remove(self.view)
        self.content_vbox.remove(self.console.view)
        self.the_game_controller = Demographic(self.window, self.console, self.content_vbox, self.filename, self.log, self.condition)
        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
    
    def observer(self,status):
        while status == True:
            print status

    def ok_button_clicked(self, widget, data=None):
        self.game_type = 'Demopgraphic'
        self.toggle_user_interaction()
        self.console.clear()
        self.playing = False
        self.is_playing = False
        self.step_forward()
        return self.game_type

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.box_control.set_sensitive(self.enabled)

    def take_turn(self):
        rs_1 = "Thank you for coming to this experiment!"
        rs_2 = "You will be asked to complete a simple task.  We will explain the detailed rules as the experiment proceeds. If you follow the instructions carefully and make good decisions, you can earn some money. The rules for payment are explained next."
        rs_3 = "After leaving this 'Welcome' page, you will be asked to answer some  questions. The instructions for the experiment will then pop up. Please, read the instructions carefully and make sure that you've understood them fully. If needed, feel free to ask the experimenter for further explanation, as subjects who ask for further clarifications usually earn more money. Finally, the experiment will start and last for about 1 hour."
        rs_4 = "At the end of the experiment you will be asked to answer a few more questions and then you will be directed to the payment screen, which summarizes your performance on the task as well as the amount of money you won."
        rs_5 = "When you are ready, hit the 'Next' button and the experiment will start." 

        s_1=textwrap.fill(rs_1,125)
        s_2=textwrap.fill(rs_2,125)
        s_3=textwrap.fill(rs_3,125)
        s_4=textwrap.fill(rs_4,125)
        s_5=textwrap.fill(rs_5,125)

        self.console.append_text("\n\n")
        self.console.append_text(s_1)
        self.console.append_text("\n\n")
        self.console.append_text(s_2)
        self.console.append_text("\n\n")
        self.console.append_text(s_3)
        self.console.append_text("\n\n")
        self.console.append_text(s_4)
        self.console.append_text("\n\n")
        self.console.append_text(s_5)
        self.console.append_text("\n\n")
        self.toggle_user_interaction()

    def delete_event(self, widget, event, data=None):
        print "Delete event called. Closing window."
        return False


class Demographic:
    def __init__(self, parent, console, mainvbox, filename, log, condition):
        self.condition = condition
        self.log = log
        self.filename = filename
        self.content_vbox = mainvbox
        self.parent = parent
        self.console = console

        self.enabled = True

        self.title = ""
        self.view = gtk.Frame(self.title)
        question_box = gtk.VBox(True, 8)
        self.view.add(question_box)

        # Biological Sex
        self.question_one = gtk.HBox(False,8)
        question_box.pack_start(self.question_one, expand=False, fill=False)
        label = gtk.Label("Biological sex: ")
        self.question_one.pack_start(label, False, False, 10)
        label.show()

        self.malebutton = gtk.RadioButton(None, "male")
        self.malebutton.connect("toggled", self.rb_callback, "male")
        self.question_one.pack_start(self.malebutton, False, False)
        self.femalebutton = gtk.RadioButton(self.malebutton, "female")
        self.femalebutton.connect("toggled", self.rb_callback, "male")
        self.question_one.pack_start(self.femalebutton, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, True, True, 5)
        separator.show()
        
        # Date of Birth
        self.question_two = gtk.HBox(False,8)
        question_box.pack_start(self.question_two, expand=False, fill=False)
        label = gtk.Label("Date of birth:")
        self.question_two.pack_start(label, False, False, 10)
        label.show()

        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, False, False, 5)

        label = gtk.Label("Day :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, False, 0)
  
        adj = gtk.Adjustment(1.0, 1.0, 31.0, 1.0, 5.0, 0.0)
        self.dayspinner = gtk.SpinButton(adj, 0, 0)
        self.dayspinner.set_wrap(True)
        b_box.pack_start(self.dayspinner, False, False, 0)
        
        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, False, False, 5)
        
        label = gtk.Label("Month :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, False, 0)

        adj = gtk.Adjustment(1.0, 1.0, 12.0, 1.0, 5.0, 0.0)
        self.monthspinner = gtk.SpinButton(adj, 0, 0)
        self.monthspinner.set_wrap(True)
        b_box.pack_start(self.monthspinner, False, False, 0)
  
        b_box = gtk.VBox(False, 0)
        self.question_two.pack_start(b_box, False, False, 5)
        
        label = gtk.Label("Year :")
        label.set_alignment(0, 0.5)
        b_box.pack_start(label, False, False, 0)
  
        adj = gtk.Adjustment(1980.0, 1900.0, 2000.0, 1.0, 100.0, 0.0)
        self.yearspinner = gtk.SpinButton(adj, 0, 0)
        self.yearspinner.set_wrap(False)
#        self.yearspinner.set_size_request(55, -1)
        b_box.pack_start(self.yearspinner, False, False, 0)

        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, False, 5)
        separator.show()
        
        # Ethnicity
        self.question_three = gtk.HBox(False,8)
        question_box.pack_start(self.question_three, expand=False, fill=False)
        
        self.ethcombobox = gtk.combo_box_new_text()
        self.question_three.pack_start(self.ethcombobox, False, False, 10)
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
        question_box.pack_start(separator, False, False, 5)
        separator.show()

        # How many GT
        self.question_four = gtk.HBox(False,8)
        question_box.pack_start(self.question_four, expand=False, fill=False)
        label = gtk.Label("How many courses have you taken that discussed game theory?")
        self.question_four.pack_start(label, False, False, 10)
        label.show()
        
        self.gt_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=20, step_incr=1))
        self.question_four.pack_start(self.gt_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, False, 5)
        separator.show()

        # How many Math
        self.question_five = gtk.HBox(False,8)
        question_box.pack_start(self.question_five, expand=False, fill=False)
        label = gtk.Label("How many mathematics and economics classes have you taken?")
        self.question_five.pack_start(label, False, False, 10)
        label.show()
        
        self.math_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=20, step_incr=1))
        self.question_five.pack_start(self.math_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # Go on
        self.question_six = gtk.HBox(False,8)
        question_box.pack_start(self.question_six, expand=False, fill=False)
        label = gtk.Label("I am ready to proceeed to the instructions:")
        self.question_six.pack_start(label, False, False, 10)
        label.show()
        
        self.button_bid = gtk.Button('Instructions')
        self.button_bid.set_size_request(95, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.question_six.pack_start(self.button_bid, False, False)
        
        self.view.show_all()
        self.toggle_user_interaction()
        self.take_turn()
        
    def rb_callback(self, widget, data=None):
        print "%s was toggled %s"

    def changed_cb(self, combobox):
        model = combobox.get_model()
        index = combobox.get_active()
        if index:
            print model[index][0]
        return

    def step_forward(self):
        self.console.clear()
        self.content_vbox.remove(self.view)
        self.the_game_controller = Instructions(self.parent, self.console, self.content_vbox, self.filename, self.log, self.condition)
        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)

    def ok_button_clicked(self, widget, data=None):
        self.log_answers()
        self.toggle_user_interaction()
        self.step_forward()

    def log_answers(self):
        # sex, dob, ethnicity, game theory classes, math classes
        demo = "Sex: " + ( "Male" if self.malebutton.get_active() else "Female") + "\n" + \
               "DOB: " + str(self.yearspinner.get_value_as_int()) + "-" + str(self.monthspinner.get_value_as_int()) + "-" + str(self.dayspinner.get_value_as_int()) + "\n" + \
               "Ethnicity: " + str(self.ethcombobox.get_model()[self.ethcombobox.get_active()][0]) + "\n" + \
               "Game theory classes: " + str(self.gt_spin_question.get_value_as_int()) + "\n" + \
               "Math classes: " + str(self.math_spin_question.get_value_as_int()) + "\n"
        self.log.writer("Answers to demographic questions:\n" + demo)

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.question_one.set_sensitive(self.enabled)
        self.question_two.set_sensitive(self.enabled)
        self.question_three.set_sensitive(self.enabled)
        self.question_four.set_sensitive(self.enabled)
        self.question_five.set_sensitive(self.enabled)
        self.question_six.set_sensitive(self.enabled)

    def take_turn(self):
        rs_1 = "Please answer to the questions above and then press 'OK'."
        s_1=textwrap.fill(rs_1,70)
        self.console.append_text(s_1)
        self.toggle_user_interaction()


class Instructions():
    def __init__(self, parent, console, mainvbox, filename, log, condition):
        self.condition = condition
        self.filename = filename
        self.log = log
        self.content_vbox = mainvbox
        self.instruction_step = 0
        self.window = parent
        self.console = console
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")

        self.title = "Instructions"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        self.content_vbox.pack_end(self.console.view)

        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.button_bid = gtk.Button("Experiment")
        self.button_bid.set_size_request(90, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.pack_end(self.button_bid, False, False)

        self.button_previous = gtk.Button('Previous')
        self.button_previous.set_size_request(80, 30)
        self.button_previous.connect("clicked", self.previous_button_clicked)
        self.box_control.pack_start(self.button_previous, False, False)

        self.button_next = gtk.Button('Next')
        self.button_next.set_size_request(80, 30)
        self.button_next.connect("clicked", self.next_button_clicked)
        self.box_control.pack_start(self.button_next, False, False)

        self.view.show_all()
        self.take_turn()

    def step_forward(self):
        self.content_vbox.remove(self.view)
        self.content_vbox.remove(self.console.view)
        self.the_game_controller = ultimatum_questions(self.window, self.console, self.content_vbox, self.filename, self.log, self.condition)
        self.content_vbox.pack_start(self.the_game_controller.view,True, True)

    def previous_button_clicked(self, widget, data=None):
        self.console.clear()
        self.instruction_step = 0
        self.take_turn()

    def next_button_clicked(self, widget, data=None):
        self.console.clear()
        self.instruction_step = 1
        self.take_turn()

    def ok_button_clicked(self, widget, data=None):
        self.console.clear()
        self.step_forward()

    def take_turn(self):
        rs_1 = "For each trial of this experiment, you will play an economic game called the 'Ultimatum Game'. Economic games are simplified settings in which 2 or more players interact, and in which the decisions they make can affect the amount of money they win. In the case of the Ultimatum Game: two players, Player 1 and Player 2, bargain on a pre-determined stake. In this experiment you will always be Player 1."

        rs_2 = "In each round of the Ultimatum Game, 10 dollars is provisionally allocated to you, and you can propose how much of this amount you and the other player is to receive.  To do this, you will indicate how much money to give to Player 2.  You can choose any amount between zero and $10.  The amount that you are to receive is simply whatever amount is left (the remaining money not allocated to Player 2)."

        rs_3 = "Once you have made a proposal, Player 2 will be given the chance to accept or reject your proposal.  If Player 2 accepts the proposal, then the amount of money will be divided as specified by you.  If Player 2 rejects the proposal, both you and Player 2 will receive zero dollars. After Player 2's decision, the interaction is over."

        rs_4 = "For instance, suppose that you decide to send $3 to Player 2. Player 2 observes the offer and decides whether to accept or not. Suppose that Player 2 rejects the offer. Then the round ends and both payoffs are $ 0. Otherwise, if Player 2 had accepted the offer, your payoff would have been $ 7 and Player 2's payoff would have been $ 3."

        rs_5 = "After you make your proposal, we will ask you to predict whether your co-player accepted or rejected that proposal. For half of the rounds, you will watch a few seconds of video recordings of your co-player, and you will be able to use the video to assist in your prediction."

        rs_6 = "The videos show 49 subjects, who participated in a similar experiment. Each subject played about 7 rounds as Player 2 (the player who receives the offer and can decide whether to accept or not). You will observe a few seconds of video from some of these rounds. Each video starts when Player 2 received your offer and ends when he/she made a decision. You will be asked to predict what the player in the video did."

        rs_7 = "Your earnings depend on both the payoffs you received in the Ultimatum Game, and the number of correct predictions you made. The maximum possible payoff for this experiment is $5. At the end of the experiment we will tell you how many correct predictions you made. If you correctly predict every decision, you will win $2. Otherwise, we will pay you in proportion to the number of correct answers you made. For instance, suppose that you get 50% of your answers correct, we will pay you $2 * 50% = $1. We will round your earnings to the top."

        rs_8 = "With respect to the Ultimatum Game payoff, we will randomly select 2 trials, sum your payoffs for those trials, and normalize the sum so that the maximum possible earning is $3. In this case we will also round the result to the top."

        rs_9 = "Your final payment will be the sum of the payment from the predictions and the payment from the Ultimatum Game. As mentioned, this payoff range is between $0 and $5."

        rs_10 = "As specified in the consent form, you will also receive 2 academic credits for participating in the experiment."

        rs_11 = "When you are ready to start the experiment, hit the 'Experiment' button. Remember that your earnings will depend on your performance on the task. Therefore, if you need any clarification, this will be the best time to ask. Otherwise, enjoy the experiment!"

        s_1=textwrap.fill(rs_1,125)
        s_2=textwrap.fill(rs_2,125)
        s_3=textwrap.fill(rs_3,125)
        s_4=textwrap.fill(rs_4,125)
        s_5=textwrap.fill(rs_5,125)
        s_6=textwrap.fill(rs_6,125)
        s_7=textwrap.fill(rs_7,125)
        s_8=textwrap.fill(rs_8,125)
        s_9=textwrap.fill(rs_9,125)
        s_10=textwrap.fill(rs_10,125)
        s_11=textwrap.fill(rs_11,125)


        if self.instruction_step == 0:
            self.console.append_text("\n\n")
            self.console.append_text(s_1)
            self.console.append_text("\n\n")
            self.console.append_text(s_2)
            self.console.append_text("\n\n")
            self.console.append_text(s_3)
            self.console.append_text("\n\n")
            self.console.append_text(s_4)
            self.console.append_text("\n\n")
            self.console.append_text(s_5)
            self.console.append_text("\n\n")
            self.console.append_text(s_6)

            self.button_bid.set_sensitive(False)
            self.button_previous.set_sensitive(False)
            self.button_next.set_sensitive(True)

        elif self.instruction_step == 1:
            self.console.append_text("\n\n")
            self.console.append_text(s_7)
            self.console.append_text("\n\n")
            self.console.append_text(s_8)
            self.console.append_text("\n\n")
            self.console.append_text(s_9)
            self.console.append_text("\n\n")
            self.console.append_text(s_10)
            self.console.append_text("\n\n")
            self.console.append_text(s_11)
            self.console.append_text("\n\n")

            self.button_bid.set_sensitive(True)
            self.button_previous.set_sensitive(True)
            self.button_next.set_sensitive(False)

    def delete_event(self, widget, event, data=None):
        print "Delete event called. Closing window."
        return False

###

class ultimatum_questions():
    def __init__(self, parent, console, mainvbox, filename, log, condition):
        self.condition = condition
        self.log = log
        self.filename = filename
        self.content_vbox = mainvbox
        self.parent = parent
        self.console = console

        self.enabled = True

        self.title = ""
        self.view = gtk.Frame(self.title)
        question_box = gtk.VBox(True, 8)
        self.view.add(question_box)

        # Understood
        self.question_one = gtk.HBox(False,8)
        question_box.pack_start(self.question_one, expand=False, fill=False)
        label = gtk.Label("Did you understoon how the ultimatum game works? ")
        self.question_one.pack_start(label, False, False, 10)
        label.show()

        self.yesbutton = gtk.RadioButton(None, "yes")
        self.yesbutton.connect("toggled", self.rb_callback, "yes")
        self.question_one.pack_start(self.yesbutton, False, False)
        self.nobutton = gtk.RadioButton(self.yesbutton, "no")
        self.nobutton.connect("toggled", self.rb_callback, "yes")
        self.question_one.pack_start(self.nobutton, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, True, True, 5)
        separator.show()

        # Question 1
        self.question_two = gtk.HBox(False,8)
        question_box.pack_start(self.question_two, expand=False, fill=False)
        label = gtk.Label("If you had to play the Ultimatum Game as Player 1, what would you offer on average?")
        self.question_two.pack_start(label, False, False, 10)
        label.show()
        
        self.q1_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=10, step_incr=1))
        self.question_two.pack_start(self.q1_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, False, 5)
        separator.show()

        # Question 2
        self.question_nine = gtk.HBox(False,8)
        question_box.pack_start(self.question_nine, expand=False, fill=False)
        label = gtk.Label("If you had to play the Ultimatum Game as Player 2, what would you expect to receive on average?")
        self.question_nine.pack_start(label, False, False, 10)
        label.show()
        
        self.q2_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=10, step_incr=1))
        self.question_nine.pack_start(self.q2_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, False, 5)
        separator.show()

        # accept 0
        self.question_three = gtk.HBox(False,8)
        question_box.pack_start(self.question_three, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 0, how many of them do you believe would have accepted the offer?")
        self.question_three.pack_start(label, False, False, 10)
        label.show()
        
        self.a0_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_three.pack_start(self.a0_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # accept 1
        self.question_four = gtk.HBox(False,8)
        question_box.pack_start(self.question_four, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 1, how many of them do you believe would have accepted the offer?")
        self.question_four.pack_start(label, False, False, 10)
        label.show()
        
        self.a1_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_four.pack_start(self.a1_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # accept 2
        self.question_five = gtk.HBox(False,8)
        question_box.pack_start(self.question_five, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 2, how many of them do you believe would have accepted the offer?")
        self.question_five.pack_start(label, False, False, 10)
        label.show()
        
        self.a2_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_five.pack_start(self.a2_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # accept 3
        self.question_six = gtk.HBox(False,8)
        question_box.pack_start(self.question_six, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 3, how many of them do you believe would have accepted the offer?")
        self.question_six.pack_start(label, False, False, 10)
        label.show()
        
        self.a3_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_six.pack_start(self.a3_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # accept 4
        self.question_seven = gtk.HBox(False,8)
        question_box.pack_start(self.question_seven, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 4, how many of them do you believe would have accepted the offer?")
        self.question_seven.pack_start(label, False, False, 10)
        label.show()
        
        self.a4_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_seven.pack_start(self.a4_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # accept 5
        self.question_eight = gtk.HBox(False,8)
        question_box.pack_start(self.question_eight, expand=False, fill=False)
        label = gtk.Label("If 100 people played the Ultimatum game, and all received an offer of $ 5, how many of them do you believe would have accepted the offer?")
        self.question_eight.pack_start(label, False, False, 10)
        label.show()
        
        self.a5_spin_question = gtk.SpinButton(adjustment=gtk.Adjustment(lower=0, upper=100, step_incr=1))
        self.question_eight.pack_start(self.a5_spin_question, False, False)
        
        separator = gtk.HSeparator()
        question_box.pack_start(separator, False, True, 5)
        separator.show()

        # Go on

        self.question_ten = gtk.HBox(False,8)
        question_box.pack_start(self.question_ten, expand=False, fill=False)
        label = gtk.Label("I am ready to start the task: ")
        self.question_ten.pack_start(label, False, False, 10)
        label.show()
        
        self.button_bid = gtk.Button("Task")
        self.button_bid.set_size_request(90, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.question_ten.pack_start(self.button_bid, False, False)
        
        self.view.show_all()
        self.toggle_user_interaction()
        self.take_turn()
        
    def rb_callback(self, widget, data=None):
        print "%s was toggled %s"

    def changed_cb(self, combobox):
        model = combobox.get_model()
        index = combobox.get_active()
        if index:
            print model[index][0]
        return

    def step_forward(self, condition):
        self.console.clear()
        self.content_vbox.remove(self.view)
        multiplayer(self.parent, self.console, self.content_vbox, self.filename, self.log, self.condition)

    def ok_button_clicked(self, widget, data=None):
        self.log_answers()
        self.toggle_user_interaction()
        self.step_forward(self.condition)

    def log_answers(self):
        demo = "Understood: " + ( "Yes" if self.yesbutton.get_active() else "No") + "\n" + \
               "Average offer make: " + str(self.q1_spin_question.get_value_as_int()) + "\n" + \
               "Expected offer: " + str(self.q2_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (0, 100): " + str(self.a0_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (1, 100): " + str(self.a1_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (2, 100): " + str(self.a2_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (3, 100): " + str(self.a3_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (4, 100): " + str(self.a4_spin_question.get_value_as_int()) + "\n" + \
               "How many accept (5, 100): " + str(self.a5_spin_question.get_value_as_int()) + "\n"
        self.log.writer("Answers to Questions on the ultimatum:\n" + demo)
        self.log.header()

    def toggle_user_interaction(self):
        self.enabled = not self.enabled
        self.question_one.set_sensitive(self.enabled)
        self.question_two.set_sensitive(self.enabled)
        self.question_three.set_sensitive(self.enabled)
        self.question_four.set_sensitive(self.enabled)
        self.question_five.set_sensitive(self.enabled)
        self.question_six.set_sensitive(self.enabled)
        self.question_seven.set_sensitive(self.enabled)
        self.question_eight.set_sensitive(self.enabled)
        self.question_nine.set_sensitive(self.enabled)
        self.question_ten.set_sensitive(self.enabled)

    def take_turn(self):
        self.toggle_user_interaction()

class Error():
    def __init__(self, parent, console, mainvbox):
        self.content_vbox = mainvbox
        self.window = parent
        self.console = console
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")

        self.title = "Wrong Argument"
        self.view = gtk.Frame(self.title)
        game_vbox = gtk.VBox(False, 8)
        self.view.add(game_vbox)
        self.content_vbox.pack_end(self.console.view)

        self.box_status = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_status, expand=False, fill=False)

        self.box_control = gtk.HBox(False, 8)
        game_vbox.pack_start(self.box_control, expand=False, fill=False)

        self.button_bid = gtk.Button('Quit')
        self.button_bid.set_size_request(80, 40)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.pack_end(self.button_bid, False, False)
        self.view.show_all()
        self.take_turn()

    def ok_button_clicked(self, widget, data=None):
        gtk.main_quit()


    def take_turn(self):
        rs_1 = "You entered the wrong argument on the command line."
        rs_3 = "The valid second argument (optional) are: 'Welcome' (default), 'Demographic', 'Instructions', 'UltimatumQuestion', 'VideoRater', 'FeaturesRater', and 'demo'. Please, press the 'Quit' button and try again."
        rs_2 = "The valid first argument (optional) are: 'facefirst' and 'facesecond'."
        rs_4 = "Note, if you run the experiment with no argument, you are prompt to a gui which helps you set up the experiment."

        s_1=textwrap.fill(rs_1,125)
        s_2=textwrap.fill(rs_2,125)
        s_3=textwrap.fill(rs_3,125)
        s_4=textwrap.fill(rs_4,125)

        self.console.append_text("\n\n")
        self.console.append_text(s_1)
        self.console.append_text("\n\n")
        self.console.append_text(s_2)
        self.console.append_text("\n\n")
        self.console.append_text(s_3)
        self.console.append_text("\n\n")
        self.console.append_text(s_4)

    def delete_event(self, widget, event, data=None):
        print "Delete event called. Closing window."
        return False

class GFPlayer:
    def __init__(self, player_id=-1, is_first=False, game_topic=''):
        self.player_id = player_id
        self.is_first = is_first
        self.game_topic = game_topic
        self.game_topic_lock = threading.Lock()


class multiplayer():
    def __init__(self, parent, console, mainvbox, filename, log, condition):
        self.capture_video = True
        self.player = GFPlayer()
        self.condition = condition
        self.filename = filename
        self.log = log
        self.content_vbox = mainvbox
        self.the_game_controller = None
#        self.enabled = True
#        self.playing = True
        self.window = parent
        self.console = console
        self.window.set_title("Beliefs and Expectations in Strategic Interaction")
        self.content_vbox.pack_end(self.console.view)

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
        if self.capture_video:
            #args = shlex.split(run_video_command)
            #self.video = subprocess.Popen(args)
            self.video = videowrapper(self.player.player_id)
            self.video.start_video()

        # Start a log for this player
#        home = os.path.expanduser('~')
#        filename = home + "/ros/ua-ros-pkg/ua_game_theory/Data/Info_Players/playerlog_" + str(self.player.player_id) + "_" + str(datetime.date.today()) + ".txt"
#        self.log.openlog(filename)

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
                    if self.capture_video:
                        self.the_game_controller = UltimatumGameController(self.window, self.console, self.player, self.log, self.video)
                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                    else:
                        self.the_game_controller = UltimatumGameController(self.window, self.console, self.player, self.log)
                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                elif game_type == "Ultimatum":
#                    if self.capture_video:
#                        self.the_game_controller = UltimatumGameController_computer(self.window, self.console, self.player, self.log, self.video)
#                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                    else:
#                        self.the_game_controller = UltimatumGameController_computer(self.window, self.console, self.player, self.log)
#                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                elif game_type == "Ultimatum with Humans & Computers":
#                    if self.capture_video:
#                        self.the_game_controller = UltimatumGameController_mix(self.window, self.console, self.player, self.log, self.video)
#                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                    else:
#                        self.the_game_controller = UltimatumGameController_mix(self.window, self.console, self.player, self.log)
#                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                elif game_type == "PaymentScreen":
#                    with gts:
#                       self.the_game_controller = Payment(self.window, self.console, self.player, self.log)
#                       self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
#                       stopvideo = True
                else:
                    with gts:
                        print game_type
                        print "Didnt match game_type: %s" %game_type
                        sys.exit(1)
                
                if self.capture_video:
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

