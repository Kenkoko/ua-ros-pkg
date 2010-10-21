#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy
import threading
import sys
#from games.TrustGame import TrustGame
#from games.Ultimatum import Ultimatum
#from games.Prisoners import Prisoners
from games import TrustGame, Ultimatum, Prisoners

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
        print "Player id: " + str(self.player_id)

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




