#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy

num_registered_players = 0
num_players = 0
registration_service = []

def register_game_player(req):
    global num_registered_players
    global num_players
    global registration_service
    num_registered_players+=1
    print "Registering player " + str(num_registered_players)
    print "num_registered_players, num_players:" + str(num_registered_players) + " " + str(num_players)
    if num_registered_players==num_players:
        #if  == :
        print "We would return"
        registration_service.shutdown()
    return RegisterGamePlayerResponse(num_registered_players)
            


def game_master_start():
    global num_registered_players
    global num_players
    global registration_service
    print "welcome to the game master"

    #rospy.init_node('game_master', anonymous=False)
    #rospy.Subscriber("game_master", String, )
    
    #while num_subscribers < num_players:
    #    # Find out how many computers are subscribed to the game_master topic
    #    print num_subscribers + "subscribed"
    #    rospy.sleep(1.0)

    # Instead of above, we will become a service that hands out ids to clients
    gameTopic = rospy.Publisher('game_master',TwoPersonGame)
    rospy.init_node('game_master')

    registration_service = rospy.Service('register_game_player',RegisterGamePlayer,register_game_player)
    print "Waiting for " + str(num_players) + " machines to check in."
    
    # We wait until the expected number signs in
    registration_service.spin()
    print "Got: " + str(num_registered_players) + " players"
    #rospy.sleep(3.0)
    
    # Now pick random assignment of players
    # Need to know:
    #  Number of experimental blocks
    #  Desired number of times a pair occurs

    num_blocks = 1
    for i in range(0,num_blocks):
        #pairing_sets = get_random_pairings_for_block(num_players)
        pairing_sets = [[1,2]]
        game_topic = 0
        print "pairing sets: " + str(pairing_sets)
        for pair in pairing_sets:
            print "pair: " + str(pair)
            game_topic+=1
            try:
                game_type = "TrustGame"
                msg = TwoPersonGame('game_topic'+str(game_topic), pair[0], pair[1], game_type)                
                print 'game_topic'+str(game_topic)
                gameTopic.publish(msg)
            except rospy.ROSInterruptException, e:
                print "something terrible happened"

        
import sys
if __name__ == '__main__':
    global num_registered_players
    global num_players
    try:
        numargs = len(sys.argv)
        print numargs
        num_players = int(sys.argv[1])
        print num_players
        game_master_start()
    except rospy.ROSInterruptException: pass


