#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
from game_faces.msg import GamePlay
import rospy

player_id = 0
#play_number = -1
is_first_player = False
current_amount = 0
pub = [];

def connect_to_master():
    global player_id
    rospy.wait_for_service('register_game_player')
    try:
        reg = rospy.ServiceProxy('register_game_player', RegisterGamePlayer)
        response = reg('Bogus IP')
        player_id = response.player_id
        print player_id
    except rospy.ServiceException, e:
        print "Could not get a player_id from master"

def play_game(gamedata):
    global player_id
    global is_first_player
    global play_number
    global pub
    # Register the game playing topic
    game_topic = gamedata.game_topic
    pub = rospy.Publisher(game_topic, GamePlay)
    is_first_player = (gamedata.first_player == player_id)
    print "is_first_player " + str(is_first_player)
    print("Inform video node which topic to watch")
    play_number = 0
    rospy.Subscriber(game_topic, GamePlay, take_turn)
    # Wait until there everyone is subscribed to this topic
    while pub.get_num_connections() < 2 :
        print "Waiting for game to start"
        print "num connections: " + str(pub.get_num_connections())
        #rospy.sleep(1.0)
    take_turn(GamePlay(0,0))

def take_turn(game_play):
    global player_id
    global is_first_player
    global current_amount
    global pub
    play_number = game_play.play_number
    if play_number == 0:
        if is_first_player:
            current_amount = 10
            offer = input("Input the first amount, between 0 and 10: ")
            pub.publish(GamePlay(1,offer))
            print "Waiting for second player"
        else:
            currrent_amount = 0
            print "Waiting for first player"
    elif play_number == 1:
        if is_first_player:
            pass    
        else:
            print "Player 1 sent "+str(game_play.amount)
            print "This was increased to "+str(game_play.amount * 3)
            offer = input("How much do you give?")
            # Check that it is legal
            current_amount = game_play.amount * 3 - offer
            print "Your payoff is now: " + str(current_amount)
            pub.publish(GamePlay(2,offer))
    elif play_number == 2:
        if is_first_player:
            print "You recieved " + str(game_play.amount) + " from the other player"
            current_amount += game_play.amount
            print "Your total payoff is: " +str(current_amount)
            pub.publish(GamePlay(3,-1))  
        else:
            pass
        print "Please wait for the next game to start"
        # Magic number, -1 stands for game finishyed

        
        

import sys
if __name__ == '__main__':
    global player_id
    try:
        print "Starting the Game Player"
        print "Connecting to Game Master"
        rospy.init_node("game_player", anonymous=True)
        rospy.Subscriber("game_master", TwoPersonGame, play_game)
        print "Training Material Here"
        print "Startup the video capture node and create topic"
        connect_to_master()
        

        rospy.spin()
    except rospy.ROSInterruptException: pass

