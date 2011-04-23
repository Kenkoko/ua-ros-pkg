import roslib
import rospy
import time
import numpy
from game_faces.msg import GamePlay
from Game import Game

prob = numpy.loadtxt('/home/robotlab/ros/ua-ros-pkg/ua_game_theory/game_faces/src/games/cprobtrust.txt')
probability = numpy.reshape(prob,(10,30))


class Sim_Ultimatum(Game):
    def __init__(self, player):
            Game.__init__(self, player)
            self.original = 10

    def take_first_turn(self):
        if not self.player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                time.sleep(numpy.random.randint(1,4,size=1))
                decision = numpy.random.multinomial(1, [.1, .15, .15, .1, .2, .3], size=1)
                # .05, .05, .1, .1, .21, .49] was this but he is too nice!!
                offer = numpy.nonzero(decision>0)[1]
                self.offer = offer
                gp = GamePlay(play_number=1,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 1:
            if not self.player.is_first_player:
                time.sleep(numpy.random.randint(1,4,size=1))
                prob_rejection = [[.73],[.64],[.49],[.20],[.07],[0],[0],[0],[0],[0],[0]]
                choice = numpy.random.binomial(1,prob_rejection[game_play.amount],size=1)
                if choice == 0:
                    self.current_amount = game_play.amount
                else:
                    self.current_amount = 0
                gp = GamePlay(play_number=2,amount=self.current_amount, player_id=self.player_id)
		# was gam_play_amount
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first_player:
                if game_play.amount > 0:
                    self.current_amount = self.original - self.offer
                else:
                    self.current_amount = 0
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        else:
            self.unregister_game()

class Sim_TrustGame(Game):
    def __init__(self, player):
            Game.__init__(self, player)
            self.current_amount = 10

    def take_first_turn(self):
        if not self.player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                time.sleep(numpy.random.randint(1,4,size=1))
                decision = numpy.random.multinomial(1, [.1, .04, .11, .1, .08, .16, .08, .07, .05, .02, .19], size=1)
                offer = numpy.nonzero(decision>0)[1]
                self.current_amount -= offer
                gp = GamePlay(play_number=1,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            else:
                currrent_amount = 0
        elif play_number == 1:
            if self.player.is_first_player:
                pass
            else:
                time.sleep(numpy.random.randint(1,4,size=1))
                recv = game_play.amount * 3
                if game_play.amount ==0:
                    offer = 0
                else:
                    print game_play.amount
                    print "Probability", probability[game_play.amount-1]
                    decision = numpy.random.multinomial(1, probability[int(game_play.amount)-1],size=1)
                    offer = numpy.nonzero(decision>0)[1]
                

                #offer = numpy.random.randint(recv+1, size=1)
                gp = GamePlay(play_number=2,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first_player:
                self.current_amount += game_play.amount
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now(); self.play_pub.publish(gp)  
            self.unregister_game()

class Sim_Prisoners(Game):
    def __init__(self, player):
        Game.__init__(self, player)
        self.payoffs = (([10,10],[12,0]),([0,12],[2,2]))
        self.row_choice = 2 # choose a number out of range so we know if we dont get a value
        self.col_choice = 2 # choose a number out of range so we know if we dont get a value

    def take_first_turn(self):
        if not self.player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        top_raw=([10, 10], [12, 0])
        bottom_raw=([0, 12], [2, 2])
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
#                time.sleep(numpy.random.randint(1,4,size=1))
                row_choice = choice = numpy.random.binomial(1,.59,size=1)
                self.row_choice = row_choice
                gp = GamePlay(play_number=1,amount=row_choice, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 1:
            if not self.player.is_first_player:
#                time.sleep(numpy.random.randint(1,4,size=1))
                col_choice = numpy.random.binomial(1,.6,size=1)
                self.col_choice = col_choice
                gp = GamePlay(play_number=2,amount=col_choice, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)

        elif play_number == 2:
            if self.player.is_first_player:
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()

class SkipInstructions(Game):
    def __init__(self, player):
        Game.__init__(self, player)

    def take_first_turn(self):
        if not self.player.is_first_player or self.player.is_first_player:
            print "this is happening"
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                print "This is also happening"
                gp = GamePlay(play_number=1,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 1:
            if self.player.is_first_player:
                print "This is also happening"
                gp = GamePlay(play_number=2,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first_player:
                print "This is also happening"
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            self.unregister_game()
            print "I am at the end"
