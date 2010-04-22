#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
import rospy
import numpy
from game_faces.msg import GamePlay


# The round robin code below is from
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

class game_master:
    def __init__(self, num_players):
        self.num_players = num_players
        self.num_registered_players = 0
        self.registration_service = []

    def register_game_player(self,req):
        # TODO: Change so that there is a map from the hostname to a unique numeric ID.
        self.num_registered_players+=1
        print "Registering player " + str(self.num_registered_players) + " named " + req.IP
        print "num_registered_players: "+ str(self.num_registered_players) + " num_players: " + str(self.num_players)
        if self.num_registered_players==self.num_players:
            self.registration_service.shutdown()
        return RegisterGamePlayerResponse(self.num_registered_players)


    def create_block(self, do_twice):
        """Call round robin,
        enable the pairs to be repeated but in reverse order,
        and randomize
        """
        pairs = roundRobin(range(1,self.num_players+1))
        if do_twice:
            pc = pairs[:]
            for p in pc:
                rp = []
                for game in p:
                    g = list(game)
                    g.reverse()
                    rp.append(tuple(g))
                pairs.append(rp)

    	numpy.random.shuffle(pairs)
        return pairs

    def start_games(self):
        """ This runs a set of two player games.  First, each player registers.
        We have predefined a set of games they will play.
        For each type of game, choose a set of matches so we have a fair tournament
        (i.e., everyone plays against everyone twice, both as first and second player, or 
        just once if there is no first/second distinction).
        Then for each game, create a topic over which game plays will be exchanged.
        Ensure that once all players get into a game, all players wait until the game is done.
        Monitor and log each game.
        """
        print "welcome to the game master"
        # We temporarily become a service that hands out ids to clients
        gameTopic = rospy.Publisher('game_master',TwoPersonGame)
        rospy.init_node('game_master')

        self.registration_service = rospy.Service('register_game_player',RegisterGamePlayer,self.register_game_player)
        print "Waiting for " + str(self.num_players) + " machines to check in."
    
        # This will spin until the expected number of players sign in
        self.registration_service.spin()
        print "Got: " + str(self.num_registered_players) + " players"

        # We are currently hand-coding in the three types of games
        game_topic = 0
        num_blocks = 3
        game_observers = []
        for i in range(0,num_blocks):
            if i == 0:
                pairs = self.create_block(True)
                game_type = "TrustGame"
            elif i == 1:
                # Note that for this game it doesn't make sense to play the same person twice
                pairs = self.create_block(False)
                game_type = "Prisoners"
            elif i == 2:
                pairs = self.create_block(True)
                game_type = "Ultimatum"
            for block_round in pairs:
                # Serve up all the games
                for pair in block_round:
                    print "pair: " + str(pair) + str(game_type)
                    game_topic+=1
                    msg = TwoPersonGame('game_topic'+str(game_topic), pair[0], pair[1], game_type)
                    the_topic = 'game_topic'+str(game_topic)

                    # Create an object for handling this game
                    observer = game_observer(msg, the_topic)
                    game_observers.append(observer)
                    gameTopic.publish(msg)
                # Wait until all pairs are finished.
                while len(game_observers) > 0 and not rospy.is_shutdown():
                    # The filter below will remove any finished game_observers
                    game_observers = filter(lambda obj:obj.still_playing, game_observers )
                    rospy.sleep(1)

        
import sys
if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print "Usage: rosrun game_faces game_master.py <num_players>"
            raise rospy.ROSInterruptException()
        gm = game_master(int(sys.argv[1]))
        gm.start_games()
    except rospy.ROSInterruptException:
        pass


