#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy
import threading
import sys
from games import Sim_TrustGame, Sim_Ultimatum, Sim_Prisoners, SkipInstructions

class autonomous_player:
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
        self.start_ros()

    def start_ros(self):
        rospy.Subscriber("game_master", TwoPersonGame, self.play_game)
        self.connect_to_master()
        print "Player id: " + str(self.player_id)
        video_topic = "Video" + str(self.player_id)
        self.video_pub = rospy.Publisher(video_topic, TwoPersonGame)

    def connect_to_master(self):
        rospy.wait_for_service('register_game_player')
        try:
            reg = rospy.ServiceProxy('register_game_player', RegisterGamePlayer)
            response = reg(str(roslib.network.get_host_name()), 0)
            self.player_id = response.player_id
            print self.player_id
        except rospy.ServiceException, e:
            print "Could not get a player_id from master"

    def play_game(self, gamedata):
        self.game_topic_lock.acquire()
        if self.game_topic is "":
            self.is_first_player = (gamedata.first_player == self.player_id)
            self.is_second_player = (gamedata.second_player == self.player_id)
            if self.is_first_player or self.is_second_player:
                self.game_topic = gamedata.game_topic
                game_type = str(gamedata.game_type)
                if game_type == "TrustGame":
                    self.the_game = Sim_TrustGame(self)
                elif game_type == "Prisoners":
                    self.the_game = Sim_Prisoners(self)
                elif game_type == "Ultimatum":
                    self.the_game = Sim_Ultimatum(self)
#                elif game_type == "TrustTutorial":
#                    self.the_game =SkipInstructions(self)
#                elif game_type == "PrisonersTutorial":
#                    self.the_game =SkipInstructions(self)
#                elif game_type == "UltimatumTutorial":
#                    self.the_game =SkipInstructions(self)
#                elif game_type == "Instructions":
#                    self.the_game =SkipInstructions(self)
#                else:
                    print "Didnt match game_type: " + str(game_type)
                self.video_pub.publish(gamedata)
                self.the_game.take_first_turn()
        else:
            print "skipped a game message from master, because I'm already in a game"
        self.game_topic_lock.release()

if __name__ == '__main__':
        game_topic_lock = threading.Lock()
        try:
                rospy.init_node("game_player", anonymous=True)
                player = autonomous_player(game_topic_lock)
                rospy.spin()
        except rospy.ROSInterruptException: pass
