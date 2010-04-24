#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
from game_faces.msg import GamePlay
import rospy
import threading
import sys
import subprocess


class Game:
    def __init__(self, player):
        self.player = player
        self.player_id = player.player_id
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=True)
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)

    def take_turn(self):
        print "take_turn must be overloaded!"
        raise rospy.ROSInterruptException()
        pass

    def take_first_turn(self):
        print "take_first_turn must be overloaded!"
        raise rospy.ROSInterruptException()
        pass
        
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_sub.unregister()
        self.play_pub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()


class TrustGame(Game):
    def __init__(self, player):
        Game.__init__(self, player)
        self.current_amount = 10
    
    def take_first_turn(self):
        if player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                offer = input("Input the first amount, between 0 and 10: ")
                self.current_amount -= offer
                gp = GamePlay(play_number=1,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
                print "Waiting for second player"
            else:
                currrent_amount = 0
                print "Waiting for first player"
        elif play_number == 1:
            if self.player.is_first_player:
                pass    
            else:
                print "Player 1 sent "+str(game_play.amount)
                print "This was increased to "+str(game_play.amount * 3)
                offer = input("How much do you give?")
                # Check that it is legal
                self.current_amount = game_play.amount * 3 - offer
                print "Your payoff is now: " + str(self.current_amount)
                #print "I am player " + str(self.player_id) + " and I am about to publish:"
                #print GamePlay(2,offer, self.player_id)
                gp = GamePlay(play_number=2,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first_player:
                print "You recieved " + str(game_play.amount) + " from the other player"
                self.current_amount += game_play.amount
                print "Your total payoff is: " +str(self.current_amount)
                #print "I am player " + str(self.player_id) + " and I am about to publish:"
                #print GamePlay(3,-1, self.player_id)
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)  
            else:
                pass
            print "Please wait for the next game to start"
            self.unregister_game()

class Prisoners(Game):
    def __init__(self, player):
        Game.__init__(self, player)
    
    def take_first_turn(self):
        if player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        top_raw=([10, 10], [12, 2])
        bottom_raw=([0, 12], [2, 2])
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
		        print top_raw
		        print bottom_raw
		        print "Your possible payoffs are on the left in each cell"
		        choice_raw = input("Please type 'U' for Upper or 'L' Lower raw:")
		        if (str(choice_raw) is not ('U')) and (str(choice_raw) is not ('L')):
			        while (str(choice_raw) is ('U')) or (str(choice_raw) is ('L')):
				        print "You entered an invalid character, please try again"
				        choice_raw = input("Please type 'U' for Upper or 'L' Lower raw:")
				gp = GamePlay(play_number=1,amount=choice_raw, player_id=self.player_id)
				gp.header.stamp = rospy.Time.now()
				self.play_pub.publish(gp)
		        print "Waiting for second player"
            else:
                print "Wait"

        elif play_number == 1:
            if self.player.is_first_player:
                print "Wait"
            else:
		        print top_raw
		        print bottom_raw
		        print "Your possible payoffs are on the right in each cell"
		        choice_column = input("Please type 'L' for Left or 'R' for Right column:")
		        gp = GamePlay(play_number=2,amount=choice_column, player_id=self.player_id)
		        gp.header.stamp = rospy.Time.now()
		        self.play_pub.publish(gp)

        elif play_number == 2:
            if str(choice_raw) is str('U'):
                if str(choice_column) is str('L'):
                    outcome = top_raw[0]
                else:
                    outcome = top_raw[1]
            else:
                if str(choice_column) is str('L'):
                    outcome = bottom_raw[0]
                else:
                    outcome = bottom_raw[1]
            if self.player.is_first_player:
                print "You chose " + str(choice_raw) + "The other player chose" + str(choice_column)
                print "Your total payoff is: " +str(outcome[0])
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            else:
                print "You chose " + str(choice_column) + "The other player chose" + str(choice_raw)
                print "Your total payoff is: " +str(outcome[1])

            self.unregister_game()
      


class game_player:
    def __init__(self, lock):
        self.game_topic_lock = lock
        self.player_id = None
        self.is_first_player = False
        self.is_second_player = False
        self.current_amount = 0
        self.video_pub = None
        self.mainthread = None
        self.game_topic = ""
        self.played_first_play = False
        self.the_game = None

        # Make the connection happen
        self.start_ros()

    def start_ros(self):
        print "Starting the Game Player"
        print "Connecting to Game Master"
        rospy.Subscriber("game_master", TwoPersonGame, self.play_game)

        print "Training Material Here"

        self.connect_to_master()
        print "PLayer id: " + str(self.player_id)

        print "We would startup the video capture node and create topic here"
        video_topic = "Video" + str(self.player_id)
        self.video_pub = rospy.Publisher(video_topic, TwoPersonGame)
        
        #run_video_command = 'rosrun game_faces game_video_capture ' + str(player_id)
        #print run_video_command
        #if capture_video:
        #    pass            
        #    # p = subprocess.Popen(run_video_command)



    def connect_to_master(self):
        rospy.wait_for_service('register_game_player')
        try:
            reg = rospy.ServiceProxy('register_game_player', RegisterGamePlayer)
            response = reg(str(roslib.network.get_host_name()))
            self.player_id = response.player_id
            print self.player_id
        except rospy.ServiceException, e:
            print "Could not get a player_id from master"

    def play_game(self, gamedata):
        # print gamedata
        # print "------"
        self.game_topic_lock.acquire()
        if self.game_topic is "":
            self.is_first_player = (gamedata.first_player == self.player_id)
            self.is_second_player = (gamedata.second_player == self.player_id)
            # print "   is_first_player " + str(self.is_first_player)
            # print "   is_second_player " + str(self.is_second_player)
            if self.is_first_player or self.is_second_player:
                # print "      I get to play!"
                # Register the game playing topic
                self.game_topic = gamedata.game_topic
                game_type = str(gamedata.game_type)
                #game_type = "TrustGame"
                if game_type == str("TrustGame"):
                    self.the_game = TrustGame(self)
                elif game_type == str("Prisoners"):
                    self.the_game = Prisoners(self)
                elif game_type == "Ultimatum":
                    self.the_game = Ultimatum(self)
                else:
                    print "Didnt match game_type: " + str(game_type)
                self.video_pub.publish(gamedata)
                self.the_game.take_first_turn()
        else:
            print "skipped a game message from master, because I'm already in a game"
        # print "Unlocking"
        # print "--"
        self.game_topic_lock.release()

if __name__ == '__main__':
    game_topic_lock = threading.Lock()
    try:
        rospy.init_node("game_player", anonymous=True)
        player = game_player(game_topic_lock)
        rospy.spin()
    except rospy.ROSInterruptException: pass




