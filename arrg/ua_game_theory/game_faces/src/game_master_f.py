#!/usr/bin/env python
import roslib; roslib.load_manifest('game_faces')
from game_faces.srv import *
from game_faces.msg import TwoPersonGame
from game_faces.msg import GamePlay
from game_faces.msg import VideoMsg
from game_faces.msg import GameSummary
import rospy
import numpy
from logger import logger
import subprocess
import os.path
import datetime
import time

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


class game_summary:
    def __init__(self):
        self.player_id = 0; self.game_type = 0; self.game_topic = 0; self.is_first_player = 0; 
        self.frame = [0,0,0,0]; self.time = [0,0,0,0]; 
        self.amount = [0,0,0,0];
        self.actionP1 = 0; self.actionP2 = 0; 
        self.payoff = 0;
        self.active = True   #Whether this line has been written to the log
    
    def addFrame(self, framenum, playnum):
        self.frame[playnum] = framenum
        #self.frame.append(framenum)
      
    def addTime(self, timestamp, playnum):
        self.time[playnum] = timestamp
        #self.time.append(timestamp)
    
    def addAmount(self, amount, playnum):
        self.amount[playnum] = amount
      
    def output(self):
        return str(self.player_id) + '\t' + str(self.game_type) + '\t' + str(self.game_topic) + '\t' + str(self.is_first_player) + '\t' + \
               str(self.frame[0]) + '\t' + str(self.frame[1]) + '\t' + str(self.frame[2]) + '\t' + str(self.frame[3]) + '\t' + \
               str(self.time[0]) + '\t' + str(self.time[1]) + '\t' + str(self.time[2]) + '\t' + str(self.time[3]) + '\t' + \
               str(self.amount[0]) + '\t' + str(self.amount[1]) + '\t' + str(self.amount[2]) + '\t' + str(self.amount[3]) + '\t' + \
               str(self.actionP1) + '\t' + str(self.actionP2) + '\t' + str(self.payoff) + '\n'
               #str(self.frame[0] if len(self.frame) > 0 else '-1') + '\t' + str(self.frame[1] if len(self.frame) > 1 else '-1') + '\t' + \
               #str(self.frame[2] if len(self.frame) > 2 else '-1') + '\t' + \
               #str(self.time[0] if len(self.time) > 0 else '-1') + '\t' + str(self.time[1] if len(self.time) > 1 else '-1') + '\t' + \
        
    
class game_summaries:
    def __init__(self):
        self.data = {}
        
        self.starttime = rospy.Time.now().secs
        
        # choose a file to log to
        filenum = 0
        while True:
            self.filename = "gamelog_" + str(datetime.date.today()) + "_" + str(filenum) + ".csv"
            filenum += 1
            if not os.path.exists(self.filename): break
        print "Logging to ", self.filename
        FILE = open(self.filename, 'w')
        FILE.write("Game Faces playing log ")
        FILE.write(time.asctime( time.localtime(time.time()) ))
        FILE.write("\n\n")
        FILE.write("PID\tGame Type\tTopic\t1st?\tf0\tf1\tf2\tf3\tt0\tt1\tt2\tt3\ta0\ta1\ta2\ta3\tact1\tact2\tpayoff\n")
        FILE.close()
    
    def processInit(self, game_topic, player_id1, player_id2, game_type):
        if not (player_id1, game_topic) in self.data:
            newdata = game_summary()
            newdata.player_id = player_id1
            newdata.is_first_player = True
            newdata.game_topic = game_topic
            newdata.game_type = game_type
            self.data[(player_id1, game_topic)] = newdata
        if not (player_id2, game_topic) in self.data:
            newdata = game_summary()
            newdata.player_id = player_id2
            newdata.is_first_player = False
            newdata.game_topic = game_topic
            newdata.game_type = game_type
            self.data[(player_id2, game_topic)] = newdata
    
    def processPlay(self, game_topic, player_id, play_number, amount, timestamp):
        if not (player_id, game_topic) in self.data:
            self.data[(player_id, game_topic)] = game_summary()
        newdata = self.data[(player_id, game_topic)]
        newdata.addTime(timestamp-self.starttime, play_number)
        if (amount >= 0):
            if (newdata.is_first_player): newdata.actionP1 = amount
            else: newdata.actionP2 = amount
        newdata.addAmount(amount, play_number)
    
    def processVideo(self, game_topic, player_id, play_number, amount, frame_number):
        if not (player_id, game_topic) in self.data:
            self.data[(player_id, game_topic)] = game_summary()
        newdata = self.data[(player_id, game_topic)]
        newdata.addFrame(frame_number, play_number)       
    
    def processSummary(self, game_topic, player_id, payoff):
        try:
            self.data[(player_id, game_topic)].payoff = payoff
            self.writeToLog(game_topic, player_id)
        except KeyError:    # Will crash on tutorials w/ computer player otherwise
            pass
    
    def writeToLog(self, game_topic, player_id):
        try:
            logdata = self.data[(player_id, game_topic)]
            if logdata.active:
                FILE = open(self.filename, 'a')
                FILE.write( logdata.output() )
                FILE.close()
                logdata.active = False
        except KeyError:
            pass
        
# The global game summary recorder
#gamelog = game_summaries()


class game_observer:
    def __init__(self, game, the_topic, single_subject, humanplayer = -1, computerplayer = -1):
        self.twopersongame = game
        self.still_playing = True
        self.turns = set()
        self.the_topic = the_topic
        self.single_subject = single_subject
        self.humanplayer = humanplayer
        self.computerplayer = computerplayer
        self.player1 = game.first_player; self.player2 = game.second_player
        self.sub = rospy.Subscriber(self.the_topic, GamePlay, self.game_play_observer)
        self.vsub = rospy.Subscriber(self.the_topic+"v", VideoMsg, self.video_observer)
        self.sumsub = rospy.Subscriber(self.the_topic+"s", GameSummary, self.summary_observer)
        gamelog.processInit(the_topic, game.first_player, game.second_player, game.game_type)
        self.sums = 0

    def game_play_observer(self, msg):
        if not self.still_playing or msg.play_number in self.turns:
            print "Callback: Got repeated message on the_topic: " + str(self.the_topic)
            print msg
        else:
            print "Callback on topic: " + str(self.the_topic)
            print "  player_id: " + str(msg.player_id)
            print "  play_number: " + str(msg.play_number)
            print "  amount: " + str(msg.amount)
            #log.log(msg)
            gamelog.processPlay(self.the_topic, msg.player_id, msg.play_number, msg.amount, msg.header.stamp.secs)
            if msg.play_number not in self.turns:
                self.turns.add(msg.play_number)
            if self.turns == set(range(4)):
                self.still_playing = False
                #gamelog.writeToLog(self.the_topic, self.player1)
                #gamelog.writeToLog(self.the_topic, self.player2)
                if single_subject:  gamelog.writeToLog(self.the_topic, self.computerplayer)
                self.sub.unregister()
                
    def video_observer(self, msg):
        print "Video callback on topic: " + str(self.the_topic+"v")
        print "  player_id: " + str(msg.player_id)
        print "  frame_number: " + str(msg.frame_number)
        print "  play_number: " + str(msg.play_number)
        print "  amount: " + str(msg.amount)
        gamelog.processVideo(self.the_topic, msg.player_id, msg.play_number, msg.amount, msg.frame_number)
    
    def summary_observer(self, msg):
        print "Summary callback on topic: " + str(self.the_topic+"s")
        print "  player_id: " + str(msg.player_id)
        print "  payoff: " + str(msg.payoff)
        gamelog.processSummary(self.the_topic, msg.player_id, msg.payoff)
        done = {self.player1: False, self.player2: False}
        done[msg.player_id] = True
        if (done[self.player1] and done[self.player2]) or (self.single_subject and done[self.humanplayer]):
            gamelog.processSummary(self.the_topic, self.computerplayer, -5)
            self.sumsub.unregister()
            self.vsub.unregister()      # Pretty safe to say the game is done; unregister subscribers


class game_master:
    def __init__(self, num_players, num_runs, single_subject):
        self.num_players = num_players
        self.num_registered_players = 0
        self.single_subject = single_subject
        self.num_runs = num_runs
        self.registration_service = []
        self.humanplayer = -1
        self.computerplayer = -1

    def handle_video_msg(msg):
        pass
    
    def handle_summary_msg(msg):
        pass

    def call_autonomous_agent(self, needed):
# This does not work yet.....
        self.needed = needed
        if self.needed:
            pid = subprocess.Popen(args =["gnome-terminal", "--command=./src/autonomous_player.py"]).pid
            #pid = subprocess.Popen(args =["gnome-terminal", "--command='rosrun game_faces autonomous_player.py'"]).pid
            print pid
        

    def register_game_player(self,req):
        # TODO: Change so that there is a map from the hostname to a unique numeric ID.
        self.num_registered_players+=1
        print "Registering " + str("human" if req.isperson else "computer") + " player " + str(self.num_registered_players) + " named " + req.IP
        print "num_registered_players: "+ str(self.num_registered_players) + " num_players: " + str(self.num_players)
        if req.isperson:
            self.humanplayer = self.num_registered_players
        else:
            self.computerplayer = self.num_registered_players
        if self.num_registered_players==self.num_players:
            self.registration_service.shutdown()
        return RegisterGamePlayerResponse(self.num_registered_players)

    # do twice logic generalized
    def create_block(self, do_n_times, is_tutorial):
        pairs = []
        if not self.single_subject and not is_tutorial:
            pairs = roundRobin(range(1,self.num_players+1))
            for i in range(do_n_times):
                pc = pairs[:]
                for p in pc:
                    rp = []
                    for game in p:
                        g = list(game)
                        if not i%2:
                            g.reverse()
                        rp.append(tuple(g))
                    pairs.append(rp)
                numpy.random.shuffle(pairs)
        elif self.single_subject and not is_tutorial:
            protot = [[(1,2)], [(2,1)]]
            for i in range(self.num_runs):
                pairs.append(protot[0])
            for i in range(do_n_times):
                for j in range(self.num_runs):
                    if i%2: pairs.append(protot[0])
                    else: pairs.append(protot[1])
            numpy.random.shuffle(pairs)
        else:
        # here individual decision making tasks have the same schedule of tutorials but repeated ...
            for i in range(do_n_times):
                par = []
                if not self.single_subject:
                    for j in range(self.num_players):
                        par.append((j+1,j+1))
                else: par.append((2,2))
                pairs.append(par)
        # This step above assumes that the real player is run second... 
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

        # We temporarily become a service that hands out ids to clients and also starts autonomous agent if needed
        gameTopic = rospy.Publisher('game_master',TwoPersonGame)
        rospy.init_node('game_master')

        global gamelog;
        gamelog = game_summaries()

        self.call_autonomous_agent(self.single_subject)
        self.registration_service = rospy.Service('register_game_player',RegisterGamePlayer,self.register_game_player)
        print "Waiting for " + str(self.num_players) + " machines to check in."

        # This will spin until the expected number of players sign in
        self.registration_service.spin()
        print "Got: " + str(self.num_registered_players) + " players"
        # We are currently hand-coding in the three types of games

        game_topic = 0
        num_blocks = 10
# num_blocks should be an iput argument: an array so that you
# select both the order and the tasks to be done by your subjects
# can do this with the gui and a check box
        game_observers = []
        for i in range(0,num_blocks):
            print i
            if i == 7:
                pairs = self.create_block(1, True)
                game_type = "Instructions"
            elif i == 1:
                pairs = self.create_block(1, True)
                game_type = "Demographic"
            elif i == 2:
                pairs = self.create_block(1, True)
                game_type = "UltimatumTutorial"
            elif i == 3:
                pairs = self.create_block(1, False)
                game_type = "Ultimatum"
            elif i == 4:
                pairs = self.create_block(1, True)
                game_type = "TrustTutorial"
                #game_type = "PaymentScreen" # debugging only
            elif i == 5:
                pairs = self.create_block(1, False)
                game_type = "TrustGame"
            elif i == 6:
                pairs = self.create_block(1, True)
                game_type = "PrisonersTutorial"
            elif i == 0:
                pairs = self.create_block(0, False)
                game_type = "Prisoners"
            elif i == 8:
                pairs = self.create_block(1, True)
                game_type = "ArmBanditTutorial"
            elif i == 9:
                pairs = self.create_block(2, True) # Should be (10, True)
                game_type = "ArmBandit"
            #elif i == 10:
            #    pairs = self.create_block(1, True)
            #    game_type = "PaymentScreen"
            for block_round in pairs:
                # Serve up all the games
                for pair in block_round:
                    print "pair: " + str(pair) + " will play " + str(game_type)
                    game_topic+=1
                    msg = TwoPersonGame('game_topic'+str(game_topic), pair[0], pair[1], game_type)
                    the_topic = 'game_topic'+str(game_topic)

                    # Create an object for handling this game
                    observer = game_observer(msg, the_topic, self.single_subject, self.humanplayer, self.computerplayer)
                    game_observers.append(observer)
                    gameTopic.publish(msg)
                # Wait until all pairs are finished.
                while len(game_observers) > 0 and not rospy.is_shutdown():
                    # The filter below will remove any finished game_observers
                    game_observers = filter(lambda obj:obj.still_playing, game_observers )
#                    for ob in game_observers:
#                        print "Which game: " + str(ob.twopersongame)
#                        print "Playing? " + str(ob.still_playing)
                    rospy.sleep(1)
        msg = TwoPersonGame('', -1, -1, 'NO_MORE_GAMES')
        gameTopic.publish(msg)

import sys
import os
import subprocess
if __name__ == '__main__':

    # Start a logger to record events and timestamps
#    log = logger()

#    filenum = 0
#    while True:
#        filename = "gamelog_" + str(datetime.date.today()) + "_" + str(filenum) + ".txt"
#        filenum += 1
#        # this is making some problem with the autonomous agent ....
#        if not os.path.exists(filename):
#            break
#    log.openlog(filename)

    try:
        single_subject = False
        if len(sys.argv) < 2:
            print "Usage: rosrun game_faces game_master.py <num_players> <num_rounds>"
            raise rospy.ROSInterruptException()
        elif int(sys.argv[1])==1:
# This set the parameters for the game in the case of single subjects experiment:
# i.e. whether it is single subject or not, how many runs per game
# Note that for now the autonomous player should be lounched first - this can be solved
# by making sure that the launcher first runs it
            single_subject = True
            gm = game_master(2, int(sys.argv[2]), single_subject)
        else:
            gm = game_master(int(sys.argv[1]),int(sys.argv[2]), single_subject)

        gm.start_games()
    except rospy.ROSInterruptException:
        pass


