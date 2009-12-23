#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import roslib
roslib.load_manifest('smartarm')

import rospy
from smartarm.msg import JointMove
from std_msgs.msg import String

class ArmMoves:

    def __init__(self):
        """
        This class publishes move requests to the ROS topic "robot/ax12/moves".
        """
        self.__clawRotated = False;
        self.lookup = {'c' : self.closeClaw,
                       'o' : self.openClaw,
                       'rc': self.rotateClaw,
                       'ac': self.armCenter,
                       'al': self.armLeft,
                       'ar': self.armRight,
                       'r' : self.raiseArm,
                       'l' : self.lowerArm,
                       'i' : self.initArm,
                       'ul': self.armRaisedLeft,
                       'll': self.armLoweredLeft,
                       'uc': self.armRaisedCenter,
                       'lc': self.armLoweredCenter,
                       'ur': self.armRaisedRight,
                       'lr': self.armLoweredRight
                      }
        #self.__publisher = rospy.Publisher('robot/ax12/moves', Move)
        self.__publisher = rospy.Publisher('smartarm/ax12/interface', JointMove)

    def closeClaw(self):
        #self.__publisher.publish(16, 725, 200)
        self.__publisher.publish(4, 212.6099706744868)

    def openClaw(self):
        #self.__publisher.publish(16, 500, 200)
        self.__publisher.publish(4, 146.62756598240469)

    def rotateClaw(self):
        if self.__clawRotated:
            #self.__publisher.publish(15, 200, 200)
            self.__publisher.publish(3, 58.651026392961882)
            self.__clawRotated = False
        else:
            #self.__publisher.publish(15, 825, 200)
            self.__publisher.publish(3, 241.93548387096774)
            self.__clawRotated = True

    def armCenter(self):
        #self.__publisher.publish(10, 512, 200)
        self.__publisher.publish(0, 150.14662756598241)

    def armLeft(self):
        #self.__publisher.publish(10, 712, 200)
        self.__publisher.publish(0, 208.7976539589443)

    def armRight(self):
        #self.__publisher.publish(10, 312, 200)
        self.__publisher.publish(0, 91.495601173020532)

    def raiseArm(self):
        #self.__publisher.publish(11, 512, 200)
        #self.__publisher.publish(12, 512, 200)
        #self.__publisher.publish(13, 412, 200)
        #self.__publisher.publish(14, 612, 200)
        self.__publisher.publish(1, 150.14662756598241)
        self.__publisher.publish(2, 120.82111436950147)

    def lowerArm(self):
        #self.__publisher.publish(11, 692, 200)
        #self.__publisher.publish(12, 332, 200)
        #self.__publisher.publish(13, 512, 200)
        #self.__publisher.publish(14, 512, 200)
        self.__publisher.publish(1, 202.9325513196481)
        self.__publisher.publish(2, 150.14662756598241)

    def initArm(self):
        #self.__publisher.publish(10, 512, 200)
        #self.__publisher.publish(11, 212, 200)
        #self.__publisher.publish(12, 812, 200)
        #self.__publisher.publish(13, 112, 200)
        #self.__publisher.publish(14, 912, 200)
        #self.__publisher.publish(15, 200, 200)
        #self.__publisher.publish(16, 500, 200)
        self.__publisher.publish(0, 150.14662756598241)
        self.__publisher.publish(1, 62.170087976539591)
        self.__publisher.publish(2, 32.84457478005865)
        self.__publisher.publish(3, 58.651026392961882)
        self.__publisher.publish(4, 146.62756598240469)
        self.__clawRotated = False

    def armRaisedLeft(self):
        self.armLeft()
        self.raiseArm()

    def armLoweredLeft(self):
        self.armLeft()
        self.lowerArm()

    def armRaisedCenter(self):
        self.armCenter()
        self.raiseArm()

    def armLoweredCenter(self):
        self.armCenter()
        self.lowerArm()

    def armRaisedRight(self):
        self.armRight()
        self.raiseArm()

    def armLoweredRight(self):
        self.armRight()
        self.lowerArm()

    def getCommandHelp(self):
        """
        Helper Method that prints the Interactive Mode command menu.
        """
        return """
Interactive Mode

    Available Commands:
        o  -  open claw
        c  -  close claw
        rc -  rotate claw
        ac -  position arm in center
        al -  position arm to the left
        ar -  position arm to the right
        r  -  raise claw
        l  -  lower claw
        i  -  inital arm pose
        ul -  equivalent to 'al' followed by 'r'
        ll -  equivalent to 'al' followed by 'l'
        uc -  equivalent to 'ac' followed by 'r'
        lc -  equivalent to 'ac' followed by 'l'
        ur -  equivalent to 'ar' followed by 'r'
        lr -  equivalent to 'ar' followed by 'l'
        """

    def handleInput(self, command):
        cmd = command.data
        print cmd
        try:
            # call function associated with command
            self.lookup[cmd]()
        except KeyError, e:
            print 'Invalid command entered'

if __name__ == '__main__':
    # Name 'ax12_arm' does not get used if this is launched from ROS launch file
    rospy.init_node('ax12_arm', anonymous=False)
    am = ArmMoves()
    rospy.set_param('smartarm/validcommands', am.lookup.keys())
    rospy.set_param('smartarm/commandhelp', am.getCommandHelp())
    rospy.Subscriber('smartarm/control', String, am.handleInput)
    print 'Arm is waiting for input.'
    rospy.spin()