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

class GFPlayer:
    def __init__(self, player_id=-1, is_first=False, game_topic=''):
        self.player_id = player_id
        self.is_first = is_first
        self.game_topic = game_topic
        self.game_topic_lock = threading.Lock()

class UltimatumGameController:
    def __init__(self, parent, shared_console, player):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        
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
        
        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=1, upper=10, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)
        
        self.view.show_all()
        self.toggle_user_interaction()
    
    def ok_button_clicked(self, widget, data=None):
        offer = self.spin_bid.get_value_as_int()
        self.set_balance(10 - offer)
        gp = GamePlay(play_number=1,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
        self.toggle_user_interaction()
        self.shared_console.append_text('You sent %d.\nWaiting for second player...\n' %offer)
    
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
            else:
                with gts:
                    self.shared_console.append_text("Waiting for first player...\n")
        elif play_number == 1:
            if self.player.is_first:
                pass   
            else:
                with gts:
                    label_prompt = gtk.Label('Player 1 has sent %d.\nYou can either accept or reject this offer.' %game_play.amount)
                    dialog = gtk.Dialog('Choose One', self.parent, gtk.DIALOG_MODAL,
                                    ('Reject', gtk.RESPONSE_REJECT,
                                      'Accept', gtk.RESPONSE_ACCEPT)) 
                    dialog.vbox.pack_start(label_prompt, True, True, 0)
                    label_prompt.show()
                    dialog.connect("delete_event", lambda w, e : None)
                    response = gtk.RESPONSE_DELETE_EVENT
                    while response == gtk.RESPONSE_DELETE_EVENT:
                        response = dialog.run()
                    dialog.destroy()
                    if response == gtk.RESPONSE_REJECT:
                        new_balance = 0
                    elif response == gtk.RESPONSE_ACCEPT:
                        new_balance = game_play.amount
                    self.set_balance(new_balance)
                    self.shared_console.append_text('Your payoff is now: %d' %new_balance)
                    gp = GamePlay(play_number=2,amount=new_balance, player_id=self.player.player_id)
                    gp.header.stamp = rospy.Time.now()
                    self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text('Your offer was ')
                    if game_play.amount == 0:
                        self.shared_console.append_text('rejected. Your payoff is now 0.\n')
                        self.set_balance(0)
                    else:
                        new_balance = 10 - game_play.amount
                        self.shared_console.append_text('accepted. Your payoff is now %d.\n' %new_balance)
                        self.set_balance(new_balance)
                time.sleep(3)
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
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

class TrustGameController:
    def __init__(self, parent, shared_console, player):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.balance = 0
        
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
        
        self.spin_bid = gtk.SpinButton(adjustment=gtk.Adjustment(lower=1, upper=10, step_incr=1))
        self.box_control.add(self.spin_bid)
        
        self.button_bid = gtk.Button(stock=gtk.STOCK_OK)
        self.button_bid.set_size_request(80, 30)
        self.button_bid.connect("clicked", self.ok_button_clicked)
        self.box_control.add(self.button_bid)
        
        self.view.show_all()
        self.toggle_user_interaction()
    
    def ok_button_clicked(self, widget, data=None):
        offer = self.spin_bid.get_value_as_int()
        self.toggle_user_interaction()
        if self.player.is_first:
            play_num = 1
            self.balance = 10 - offer
            self.set_balance(self.balance)
            self.shared_console.append_text('You sent %d.\nYour current payoff is %d\nWaiting for second player...\n' %(offer, self.balance))
        else:
            play_num = 2
            self.set_balance(self.valueX3 - offer)
            self.shared_console.append_text('You returned %d.\nYour current payoff is %d\n' %(offer, self.valueX3 - offer))
        gp = GamePlay(play_number=play_num,amount=offer, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
    
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
                    s = 'Player 1 has sent %d. The tripled value is %d.\nChoose an amount to return to player 1 and press OK.\n' %(value, self.valueX3)
                    self.shared_console.append_text(s)
                    self.spin_bid.get_adjustment().set_lower(0)
                    self.spin_bid.get_adjustment().set_upper(self.valueX3)
                    self.spin_bid.set_value(0)
                    self.set_balance(self.valueX3)
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                with gts:
                    self.shared_console.append_text('Player 2 returned %d.\n' %game_play.amount)
                    new_balance = self.balance + game_play.amount
                    self.shared_console.append_text('Your payoff is now %d.\n' %new_balance)
                    self.set_balance(new_balance)
                time.sleep(3)
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
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

class PrisonersGameController:
    def __init__(self, parent, shared_console, player):
        self.parent = parent
        self.shared_console = shared_console
        self.player = player
        self.balance = 0
        self.payoffs = (([10,10],[12,2]),([2,12],[2,2]))
        self.row_choice = 2 # choose a number out of range so we know if we dont get a value
        self.col_choice = 2 # choose a number out of range so we know if we dont get a value
        
        self.enabled = True
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=(not self.player.is_first))
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)
        
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

        table_cells = []

        if self.player.is_first:
            nrows = 2
            ncols = 3
            
            self.button_top = gtk.CheckButton("Top")
            self.button_top.connect("toggled", self.checkbox_toggled, "Top")
            table_cells.append(self.button_top)
            
            self.button_bottom = gtk.CheckButton("Bottom")
            self.button_bottom.connect("toggled", self.checkbox_toggled, "Bottom")
            table_cells.append(self.button_bottom)
        else:
            nrows = 3
            ncols = 2
            
            self.button_left = gtk.CheckButton("Left")
            self.button_left.connect("toggled", self.checkbox_toggled, "Left")
            table_cells.append(self.button_left)
            
            self.button_right = gtk.CheckButton("Right")
            self.button_right.connect("toggled", self.checkbox_toggled, "Right")
            table_cells.append(self.button_right)
        
        self.table = gtk.Table(rows=nrows, columns=ncols, homogeneous=False)
        
        self.label_tl = gtk.Label("10 | 10")
        table_cells.append(self.label_tl)
        
        self.label_tr = gtk.Label("12 | 2")
        table_cells.append(self.label_tr)
        
        self.label_bl = gtk.Label("2 | 12")
        table_cells.append(self.label_bl)
        
        self.label_br = gtk.Label("2 | 2")
        table_cells.append(self.label_br)
        
        table_cells.reverse()
        
        if self.player.is_first:
            i_max = ncols
            j_max = nrows
        else:
            i_max = nrows
            j_max = ncols
        for i in xrange(i_max):
            for j in xrange(j_max):
                if self.player.is_first:
                    self.table.attach(table_cells.pop(), i, i + 1, j, j + 1, xoptions=gtk.EXPAND)
                else:
                    self.table.attach(table_cells.pop(), j, j + 1, i, i + 1, xoptions=gtk.EXPAND)
        
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
            self.shared_console.append_text('Waiting for second player...\n')
        else:
            play_num = 2
            amt_val = self.col_choice
            r_str = ('top', 'bottom')[self.row_choice]
            c_str = ('left', 'right')[self.col_choice]
            self.shared_console.append_text('You chose the %s column and player 1 chose the %s row.\n' %(c_str, r_str))
            new_balance = self.payoffs[self.row_choice][self.col_choice][self.player.is_first]
            self.set_balance(new_balance)
        gp = GamePlay(play_number=play_num,amount=amt_val, player_id=self.player.player_id)
        gp.header.stamp = rospy.Time.now()
        self.play_pub.publish(gp)
    
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
                    self.toggle_user_interaction()
                    self.shared_console.append_text("Your possible payoffs are on the left in each cell.\n")
                    self.shared_console.append_text("Please choose the top row or the bottom row and press OK.\n")
            else:
                with gts:
                    self.shared_console.append_text("Waiting for first player...\n")
        elif play_number == 1:
            if self.player.is_first:
                pass
            else:
                with gts:
                    self.row_choice = game_play.amount
                    self.shared_console.append_text("Your possible payoffs are on the right in each cell.\n")
                    self.shared_console.append_text("Please choose the left column or the right column and press OK.\n")
                    self.toggle_user_interaction()
        elif play_number == 2:
            if self.player.is_first:
                with gts:
                    self.col_choice = game_play.amount
                    r_str = ('top', 'bottom')[self.row_choice]
                    c_str = ('left', 'right')[self.col_choice]
                    self.shared_console.append_text('You chose the %s row and player 2 chose the %s column.\n' %(r_str, c_str))
                    new_balance = self.payoffs[self.row_choice][self.col_choice][self.player.is_first]
                    self.shared_console.append_text('Your payoff is now %d.\n' %new_balance)
                    self.set_balance(new_balance)
                time.sleep(3)
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player.player_id)
                gp.header.stamp = rospy.Time.now()
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
        self.text_buffer.set_text('')

class GameFacesUI:
    def __init__(self):
        # ivars
        self.player = GFPlayer()
        self.the_game_controller = None
    
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("destroy", lambda w: gtk.main_quit())
        self.window.set_title("Game")
        self.window.resize(500, 500)

        self.content_vbox = gtk.VBox(False, 8)
        self.content_vbox.set_border_width(10)
        self.window.add(self.content_vbox)
        
        self.console = GFConsoleController()
        self.content_vbox.pack_end(self.console.view)
        
        self.window.show_all()
        self.register_player()
        
    def register_player(self):
        self.console.append_text("Connecting to Game Master...\n")
        rospy.init_node('game_faces_ui', anonymous=True)
        self.game_server_sub = rospy.Subscriber("game_master", TwoPersonGame, self.play_game)

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
        gts = GtkThreadSafe()
        if gamedata.game_type == 'NO_MORE_GAMES':
            self.console.append_text('\n\nNo more games!\n')
            return
        self.player.game_topic_lock.acquire()
        if self.player.game_topic is '':
            is_first_player = (gamedata.first_player == self.player.player_id)
            is_second_player = (gamedata.second_player == self.player.player_id)
            if is_first_player or is_second_player:
                if self.the_game_controller:
                    self.content_vbox.remove(self.the_game_controller.view)
                self.console.clear()
                self.console.append_text("\nStarting game: %s\n\n\n" %gamedata.game_type)
                # Register the game playing topic
                game_topic = gamedata.game_topic
                self.player.is_first = is_first_player
                self.player.game_topic = game_topic
                game_type = str(gamedata.game_type)
                if game_type == "TrustGame":
                    with gts:
                        self.the_game_controller = TrustGameController(self.window, self.console, self.player)
                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                elif game_type == "Prisoners":
                    with gts:
                        self.the_game_controller = PrisonersGameController(self.window, self.console, self.player)
                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                elif game_type == "Ultimatum":
                    with gts:
                        self.the_game_controller = UltimatumGameController(self.window, self.console, self.player)
                        self.content_vbox.pack_start(self.the_game_controller.view, expand=False)
                else:
                    with gts:
                        print "Didnt match game_type: %s" %game_type
                        sys.exit(1)
                self.the_game_controller.take_first_turn()
        else:
            with gts:
                print "skipped a game message from master, because I'm already in a game"
        self.player.game_topic_lock.release()

if __name__ == "__main__":
    g = GameFacesUI()
    gtk.main()
