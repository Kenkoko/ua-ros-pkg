import roslib
import rospy
from game_faces.msg import GamePlay
from Game import Game

class Prisoners(Game):
    def __init__(self, player):
        Game.__init__(self, player)
        self.choice_raw = -1        
        self.choice_column = -1
    
    def take_first_turn(self):
        if self.player.is_first_player:
            gp = GamePlay(play_number=0,amount=0, player_id=self.player_id)
            gp.header.stamp = rospy.Time.now()
            self.play_pub.publish(gp)

    def take_turn(self, game_play):
        top_raw=([10, 10], [12, 2])
        bottom_raw=([0, 12], [2, 2])
        play_number = game_play.play_number
        if play_number == 0:
            if self.player.is_first_player:
                print top_raw
                print bottom_raw
                print "Your possible payoffs are on the left in each cell"
                choice_raw = raw_input("Please type 'U' for Upper or 'L' Lower raw:")
                while not (str(choice_raw) == 'U' or str(choice_raw) == 'L'):
                    print "You entered an invalid character, please try again"
                    choice_raw = raw_input("Please type 'U' for Upper or 'L' Lower raw:")
                # NOTE: We cast the character to the ASCII integer using "ord" because
                # GamePlay.amount is defined as an integer
                self.choice_raw = choice_raw
                gp = GamePlay(play_number=1,amount=ord(choice_raw), player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
                print "Waiting for second player"
            else:
                print "Wait"
                
        elif play_number == 1:
            if self.player.is_first_player:
                print "Wait"
            else:
                print top_raw
                print bottom_raw
                print "Your possible payoffs are on the right in each cell"
                choice_column = raw_input("Please type 'L' for Left or 'R' for Right column:")
                self.choice_column = choice_column
                gp = GamePlay(play_number=2,amount=ord(choice_column), player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)

        elif play_number == 2:
            # Remember that the values are integers, so we cast them back to chars
            # for human readability
            if self.player.is_first_player:
                choice_raw = self.choice_raw
                choice_column = chr(game_play.amount)
            else:
                choice_raw = chr(game_play.amount)
                choice_column = self.choice_column
                
            if choice_raw == 'U':
                if choice_column == 'L':
                    outcome = top_raw[0]
                else:
                    outcome = top_raw[1]
            else:
                if choice_column == 'L':
                    outcome = bottom_raw[0]
                else:
                    outcome = bottom_raw[1]
            if self.player.is_first_player:
                print "You chose " + choice_raw + "The other player chose" + choice_column
                print "Your total payoff is: " +str(outcome[0])
                gp = GamePlay(play_number=3,amount=-1, player_id=self.player_id)
                gp.header.stamp = rospy.Time.now()
                self.play_pub.publish(gp)
            else:
                print "You chose " + choice_column + "The other player chose" + choice_raw
                print "Your total payoff is: " +str(outcome[1])

            self.unregister_game()
