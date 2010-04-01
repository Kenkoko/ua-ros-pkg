#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from RegisterGamePlayer.srv import *
import rospy

num_registered_players = 0
num_players = 0

def register_game_player(req):
    num_registered_players+=1
    print "Registering player " + str(num_registered_players)
    return RegisterGamePlayerResponse(num_registered_players)
    if num_registered_players == num_players:
        raise BaseException


def game_master_start():
    print "welcome to the game master"

    #rospy.init_node('game_master', anonymous=False)
    #rospy.Subscriber("game_master", String, )
    
    #while num_subscribers < num_players:
    #    # Find out how many computers are subscribed to the game_master topic
    #    print num_subscribers + "subscribed"
    #    rospy.sleep(1.0)

    # Instead of above, we will become a service that hands out ids to clients
    rospy.init_node('game_master')
    s = rospy.Service('register_game_player',register_game_player)
    print "Waiting for " + num_players + " machines to check in."
    try:
        rospy.spin()
    except:
        print "Got: " + str(num_registered_players) + "players"
    
    # Now we have all game players waiting for instructions
    # register their game topics
    for j in range(num_players):
        rospy.wait_for_service('game_player'+str(j))
        game_register[j] = rospy.ServiceProxy('register_game',RegisterGamePlayerTopic)


    # Create the topics for pairwise games to play on
    # Is this really necessary?
    #for i in num_players / 2:
    #    pub = rospy.Publisher('pair' + str(i), IntegerArray)

    # Run some exercise training protocol
    
    # Now pick random assignment of players
    # Need to know:
    #  Number of experimental blocks
    #  Desired number of times a pair occurs
    for i in num_blocks:
        pairing_sets = get_random_pairings_for_block(num_players)
        game_topic = 0
        for pair in pairing_sets:
            game_topic+=1
            try:
                status = game_register[pair[0]](game_topic,1)
                status = game_register[pair[1]](game_topic,2)
            except rospy.ServiceException, e:
                print "something terrible happened"

if __name__ == '__main__':
    try:
        num_players = arg[1]
        game_master_start()
    except rospy.ROSInterruptException: pass


