import roslib
import rospy
from game_faces.msg import GamePlay
#import make_tutorial

class Game:
    def __init__(self, player):
        self.player = player
        self.player_id = player.player_id
        self.play_pub = rospy.Publisher(self.player.game_topic, GamePlay, subscriber_listener=None, tcp_nodelay=True, latch=True)
        self.play_sub = rospy.Subscriber(self.player.game_topic, GamePlay, self.take_turn)

#    def tutorial(self):

    def take_turn(self):
        print "take_turn must be overloaded!"
        raise rospy.ROSInterruptException()

    def take_first_turn(self):
        print "take_first_turn must be overloaded!"
        raise rospy.ROSInterruptException()

    def get_payoff(self):
        return self.payoff
        
    def unregister_game(self):
        self.player.game_topic_lock.acquire()
        self.play_sub.unregister()
        self.play_pub.unregister()
        self.player.game_topic = ""
        self.player.game_topic_lock.release()


