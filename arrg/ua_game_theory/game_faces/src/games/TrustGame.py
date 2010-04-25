import roslib
import rospy
from game_faces.msg import GamePlay
from Game import Game

class TrustGame(Game):
    def __init__(self, player):
        Game.__init__(self, player)
        self.current_amount = 10
    
    def take_first_turn(self):
        if self.player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)
        
    def take_turn(self, game_play):
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                offer = input("Input the first amount, between 0 and 10: ")
                self.current_amount -= offer
                gp = GamePlay(play_number=1,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
                print "Waiting for second player"
            else:
                currrent_amount = 0
                print "Waiting for first player"
        elif play_number == 1:
            if self.player.is_first_player:
                pass    
            else:
                print "Player 1 sent "+str(game_play.amount)
                print "This was increased to "+str(game_play.amount * 3)
                offer = input("How much do you give?")
                # Check that it is legal
                self.current_amount = game_play.amount * 3 - offer
                print "Your payoff is now: " + str(self.current_amount)
                #print "I am player " + str(self.player_id) + " and I am about to publish:"
                #print GamePlay(2,offer, self.player_id)
                gp = GamePlay(play_number=2,amount=offer, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
        elif play_number == 2:
            if self.player.is_first_player:
                print "You recieved " + str(game_play.amount) + " from the other player"
                self.current_amount += game_play.amount
                print "Your total payoff is: " +str(self.current_amount)
                #print "I am player " + str(self.player_id) + " and I am about to publish:"
                #print GamePlay(3,-1, self.player_id)
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)  
            else:
                pass
            print "Please wait for the next game to start"
            self.unregister_game()
