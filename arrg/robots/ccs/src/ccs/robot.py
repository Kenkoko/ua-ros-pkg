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
roslib.load_manifest('ccs')

import rospy
from ax12.msg import Move
from std_msgs.msg import String

class Robot:

    def __init__(self):
        """
        This class publishes move requests to the ROS topic "robot/ax12/moves".
        """
        self.__canUseRFID = False
        self.__canUseClaw = False
        self.__rotated = False;
        self.lookup = {'c':self.closeClaw,
                       'o':self.openClaw,
                       'r':self.raiseClaw,
                       'l':self.lowerClaw,
                       'rc':self.rotateClaw,
                       'f':self.moveForward,
                       'b':self.moveBackward,
                       's':self.stop,
                       'pr':self.parkRFIDArm,
                       'pc':self.parkClawArm,
                       'ur':self.positionRFIDArmForUse,
                       'uc':self.positionClawArmForUse,
                       'sc':self.scanAroundWithRFID,
                       'mc':self.moveClawToSide,
                       'tr':self.turnToRight,
                       'tl':self.turnToLeft
                     }
        self.__publisher = rospy.Publisher('robot/ax12/moves', Move)

    def moveBackward(self):
        """
        Sets the robot into backwards motion at approximately 3/5 stop speed.
        """
        self.__publisher.publish(1, 0, 300)
        self.__publisher.publish(2, 0, -300)
        self.__publisher.publish(3, 0, 300)
        self.__publisher.publish(4, 0, -300)

    def moveForward(self):
        """
        Sets the robot into forward motion at approximately 3/5 stop speed.
        """
        self.__publisher.publish(1, 0, -300)
        self.__publisher.publish(2, 0, 300)
        self.__publisher.publish(3, 0, -300)
        self.__publisher.publish(4, 0, 300)

    def stop(self):
        """
        Stops the robot's wheels.
        """       
        self.__publisher.publish(1, 0, 0)
        self.__publisher.publish(2, 0, 0)
        self.__publisher.publish(3, 0, 0)
        self.__publisher.publish(4, 0, 0)

    def parkRFIDArm(self):
        """
        Returns the RFID arm to its original parked position.
        """        
        self.__publisher.publish(10, 474, 300)
        self.__publisher.publish(9, 794, 300)
        self.__publisher.publish(11, 343, 300)
        self.__canUseRFID = False

    def parkClawArm(self):
        """
        Returns the claw arm to its original parked position.
        """
        self.__publisher.publish(12, 1023, 200)
        self.__publisher.publish(8, 850, 200)
        self.__publisher.publish(6, 650, 200)
        self.__publisher.publish(7, 380, 200)
        self.__publisher.publish(17, 175, 200)

        self.__rotated = False;
        self.__canUseClaw = False

    def positionClawArmForUse(self):
        """
        Positions the claw arm out in front of the robot, such that
        it can be used to interact with objects.
        """
        self.parkRFIDArm()        
        self.__canUseClaw = True
        self.lowerClaw()
        self.openClaw()
        self.__publisher.publish(17, 175, 200) # Initialize claw rotation


    def positionRFIDArmForUse(self):
        """
        Positions the RFID arm out in front of the robot, such that
        it can be used to interact with objects.
        """
        self.parkClawArm()        
        self.__publisher.publish(11, 660, 300)
        self.__publisher.publish(9, 620, 300)
        self.__publisher.publish(10, 167, 300)
        self.__canUseRFID = True

    def scanAroundWithRFID(self):
        """
        Simulate a scanning motion with the RFID reader. This will only work if the RFID arm is
        positioned for use and the claw arm is parked.
        """
        if self.__canUseRFID:            
            self.__publisher.publish(11, 512, 100)
            rospy.sleep(1)
            self.__publisher.publish(11, 812, 100)
            rospy.sleep(1)
            self.__publisher.publish(11, 660, 100)

    def moveClawToSide(self):
        """
        Raise the claw and position it to the left of the robot. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.raiseClaw()            
            self.__publisher.publish(12, 960, 150)

    def openClaw(self):
        """
        Opens the claw to approximately 6" wide. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:            
            self.__publisher.publish(6, 512, 300)
            self.__publisher.publish(7, 512, 300)

    def closeClaw(self):
        """
        Closes the claw such that the 1" wide phidgets accelerometer box can be moved. This will
        only work if the claw arm is positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:            
            self.__publisher.publish(6, 601, 300)
            self.__publisher.publish(7, 437, 300)

    def raiseClaw(self):
        """
        Raises the claw such that the object is approximately positioned above the robot's center of
        mass. This will only work if the claw arm is positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:            
            self.__publisher.publish(8, 541, 200)

    def putClawInFront(self):
        self.__publisher.publish(12, 660, 200)
        

    def lowerClaw(self):
        """
        Lowers the claw to the approximately 1.5" off the ground. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:            
            self.__publisher.publish(8, 463, 200)
            self.putClawInFront()

    def rotateClaw(self):
        """
        Rotates the claw. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            print "rotated: " + str(self.__rotated)
            if self.__rotated:
                self.raiseClaw()
                self.__publisher.publish(17, 175, 200)
                self.__rotated = not self.__rotated
            else:
                self.raiseClaw()
                self.__publisher.publish(17, 785, 200)
                self.__rotated = not self.__rotated

    def turnToRight(self):
        """
        Starts the robot wheels such that the robot spins clockwise.
        """        
        self.__publisher.publish(1, 0, 300)
        self.__publisher.publish(2, 0, 300)
        self.__publisher.publish(3, 0, 300)
        self.__publisher.publish(4, 0, 300)

    def turnToLeft(self):
        """
        Starts the robot wheels such that the robot spins counter clockwise.
        """ 
        self.__publisher.publish(1, 0, -300)
        self.__publisher.publish(2, 0, -300)
        self.__publisher.publish(3, 0, -300)
        self.__publisher.publish(4, 0, -300)

    def getCommandHelp(self):
        """
        Helper Method that prints the Interactive Mode command menu.
        """
        return """
Interactive Mode

    Available Commands:
        f  -  move forward
        b  -  move backward
        tr -  continuous turn right
        tl -  continuous turn left
        s  -  stop any wheel motion
        uc -  position claw for use
        o  -  open claw
        c  -  close claw
        r  -  raise claw
        l  -  lower claw
        rc -  rotate claw
        mc -  raise and move to side
        pc -  park claw
        ur -  position RFID arm for use
        sc -  scan around
        pr -  park RFID arm
        x  -  reset
        """

    def handleInput(self, command):
        cmd = command.data
        print cmd
        try:
            # call function associated with command
            self.lookup[cmd]()
        except KeyError, e:
            if cmd == 'x':
                self.parkClawArm()
                self.parkRFIDArm()
                self.stop();
            else:
                print 'Invalid command entered'

if __name__ == '__main__':
    rospy.init_node('robot', anonymous=False)
    r = Robot()
    r.parkClawArm()
    r.parkRFIDArm()
    rospy.set_param('robot/validcommands', r.lookup.keys())
    rospy.set_param('robot/commandhelp', r.getCommandHelp())
    rospy.Subscriber('woz/robotcontrol', String, r.handleInput)
    print 'Robot is waiting for input.'
    rospy.spin()