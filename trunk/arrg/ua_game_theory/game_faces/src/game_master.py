#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy
import numpy

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

# Below round robin code from
## {{{ http://code.activestate.com/recipes/65200/ (r1)
def roundRobin(units, sets=None):
    """ Generates a schedule of "fair" pairings from a list of units """
    if len(units) % 2:
        units.append(None)
    count    = len(units)
    sets     = sets or (count - 1)
    half     = count / 2
    schedule = []
    for turn in range(sets):
        pairings = []
        for i in range(half):
            pairings.append((units[i], units[count-i-1]))
        units.insert(1, units.pop())
        schedule.append(pairings)
    return schedule
 
def create_block(num_players, do_twice):
    pairs = roundRobin(range(num_players))
    if do_twice:
        pc = pairs[:]
        for p in pc:
            rp = []
            for game in p:
                g = list(game)
                g.reverse()
                rp.append(tuple(g))
            pairs.append(rp)
        
    for pairings in pairs:
        print pairings
    
    return pairs

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

    num_blocks = 3
    for i in range(num_blocks):
        if i == 0:
            pairs = create_block(num_players, True):
            game_type = "TrustGame"
        if i == 1:
            pairs = create_block(num_players, False):
            game_type = "Prisoners"
        if i == 2:
            pairs = create_block(num_players, True):
            game_type = "Ultimatum"

        for block_round in pairs
            game_topic = 0
            for pair in block_round:
                print "pair: " + str(pair)
                game_topic+=1
                try:
                    msg = TwoPersonGame('game_topic'+str(game_topic), pair[0], pair[1], game_type)                
                    print 'game_topic'+str(game_topic)
                    gameTopic.publish(msg)
                except rospy.ROSInterruptException, e:
                    print "something terrible happened"
            # Some logic here to wait until all the games finish playing

        
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


