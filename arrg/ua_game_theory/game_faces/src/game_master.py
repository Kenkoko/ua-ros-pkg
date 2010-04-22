#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy
import numpy
from game_faces.msg import GamePlay

num_registered_players = 0
num_players = 0
registration_service = []

class game_observer:
    def __init__(self, game, the_topic):
        self.twopersongame = game
        self.still_playing = True
        self.turns = set()
        self.the_topic = the_topic
        self.sub = rospy.Subscriber(self.the_topic, GamePlay, self.game_play_observer)

    def game_play_observer(self, msg):
        print "Callback: the_topic: " + str(self.the_topic)
        print "Game play: " + str(msg)
        
        if msg.play_number not in self.turns:
            self.turns.add(msg.play_number)
        
        if self.turns == set(range(4)):
            self.still_playing = False
            self.sub.unregister()


def register_game_player(req):
    global num_registered_players
    global num_players
    global registration_service

    # Change so that there is a map from the IP message to a unique numberic ID.

    num_registered_players+=1
    print "Registering player " + str(num_registered_players) + " named " + req.IP
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
    pairs = roundRobin(range(1,num_players+1))
    if do_twice:
        pc = pairs[:]
        for p in pc:
            rp = []
            for game in p:
                g = list(game)
                g.reverse()
                rp.append(tuple(g))
            pairs.append(rp)
            print "Reversed pair: " + str(rp)

	numpy.random.shuffle(pairs)
    print "Randomized Pairs: "
    for p in pairs:
        print p
    return pairs

	#order=range(0,len(pairs))
	#numpy.random.shuffle(order)
	#randomized_block=[]
	
	#for i in range(0,len(pairs)):
	#	randomized_block.append(pairs[order[i]])

    #print "Randomized Pairs: "
    #for p in randomized_block:
    #    print p
        
	#return randomized_block

def game_master_start():
    global num_registered_players
    global num_players
    global registration_service
    print "welcome to the game master"

    # Instead of above, we will become a service that hands out ids to clients
    gameTopic = rospy.Publisher('game_master',TwoPersonGame)
    rospy.init_node('game_master')

    registration_service = rospy.Service('register_game_player',RegisterGamePlayer,register_game_player)
    print "Waiting for " + str(num_players) + " machines to check in."
    
    # We wait until the expected number signs in
    registration_service.spin()
    print "Got: " + str(num_registered_players) + " players"
    

    game_topic = 0
    num_blocks = 3
    game_observers = []
    for i in range(0,num_blocks):
        if i == 0:
            pairs = create_block(num_players, True)
            game_type = "TrustGame"
        if i == 1:
            pairs = create_block(num_players, False)
            game_type = "Prisoners"
        if i == 2:
            pairs = create_block(num_players, True)
            game_type = "Ultimatum"
        print game_type
        for block_round in pairs:
            for pair in block_round:
                print "pair: " + str(pair)
                game_topic+=1
                try:
                    msg = TwoPersonGame('game_topic'+str(game_topic), pair[0], pair[1], game_type)
                    the_topic = 'game_topic'+str(game_topic)
                    observer = game_observer(msg, the_topic)
                    game_observers.append(observer)
                    gameTopic.publish(msg)
                    if len(game_observers) == num_players / 2:
                        while len(game_observers) > 0 and not rospy.is_shutdown():
                            #print "Outer loop: outstanding_games: " + str(len(game_observers))
                            game_observers = filter(lambda obj:obj.still_playing, game_observers )
                            rospy.sleep(1)
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


