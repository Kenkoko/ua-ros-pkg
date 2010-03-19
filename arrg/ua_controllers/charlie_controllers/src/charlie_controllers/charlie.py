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
roslib.load_manifest('charlie_controller')

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class CharlieRobot:

    def __init__(self):
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

        rospy.Subscriber('woz/robotcontrol', String, self.handleInput)

        self.drivetrain_pubs = {'right_front_wheel_controller': None,
                                'left_front_wheel_controller': None,
                                'right_rear_wheel_controller': None,
                                'left_rear_wheel_controller': None}

        self.rfidarm_pubs = {'shoulder_pan_controller': None,
                             'elbow_tilt_controller': None,
                             'wrist_tilt_controller': None}

        self.clawarm_pubs = {'shoulder_pan_controller': None,
                             'elbow_tilt_controller': None,
                             'wrist_rotate_controller': None,
                             'finger_left_controller': None,
                             'finger_right_controller': None}

        for pub in (self.drivetrain_pubs, self.rfidarm_pubs, self.clawarm_pubs):
            for k in pub:
                pub[k] = rospy.Publisher(k + '/command', Float64)

    def moveBackward(self):
        """
        Sets the Charlie Robot into backwards motion at approximately 4 rads/sec.
        """
        self.drivertrain_pubs['right_front_wheel_controller'].publish(4)
        self.drivertrain_pubs['left_front_wheel_controller'].publish(-4)
        self.drivertrain_pubs['right_rear_wheel_controller'].publish(4)
        self.drivertrain_pubs['left_rear_wheel_controller'].publish(-4)

    def moveForward(self):
        """
        Sets the Charlie Robot into forward motion at approximately 4 rads/sec.
        """
        self.drivertrain_pubs['right_front_wheel_controller'].publish(-4)
        self.drivertrain_pubs['left_front_wheel_controller'].publish(4)
        self.drivertrain_pubs['right_rear_wheel_controller'].publish(-4)
        self.drivertrain_pubs['left_rear_wheel_controller'].publish(4)

    def stop(self):
        """
        Stops the Charlie robot's wheels.
        """
        self.drivertrain_pubs['right_front_wheel_controller'].publish(0)
        self.drivertrain_pubs['left_front_wheel_controller'].publish(0)
        self.drivertrain_pubs['right_rear_wheel_controller'].publish(0)
        self.drivertrain_pubs['left_rear_wheel_controller'].publish(0)

    def parkRFIDArm(self):
        """
        Returns the RFID arm to its original parked position.
        """
        self.rfidarm_pubs['wrist_tilt_controller'].publish(-0.2147572980)
        self.rfidarm_pubs['elbow_tilt_controller'].publish(1.44194185800)
        self.rfidarm_pubs['shoulder_pan_controller'].publish(-1.65158588)
        self.__canUseRFID = False

    def parkClawArm(self):
        """
        Returns the claw arm to its original parked position.
        """
        self.clawarm_pubs['shoulder_pan_controller'].publish(1.825437033)
        self.clawarm_pubs['elbow_tilt_controller'].publish(1.71805838400)
        self.clawarm_pubs['finger_left_controller'].publish(-0.705631122)
        self.clawarm_pubs['finger_right_controller'].publish( -0.6749515)
        self.clawarm_pubs['wrist_rotate_controller'].publish(0.000000000)

        self.__rotated = False;
        self.__canUseClaw = False

    def positionClawArmForUse(self):
        """
        Positions the claw arm out in front of the robot, such that
        it can be used to interact with objects.
        """
        self.parkRFIDArm()
        self.__canUseClaw = True
        self.__rotated = False;
        self.lowerClaw()
        self.openClaw()
        # Initialize claw rotation
        self.clawarm_pubs['wrist_rotate_controller'].publish(0.0)


    def positionRFIDArmForUse(self):
        """
        Positions the RFID arm out in front of the robot, such that
        it can be used to interact with objects.
        """
        self.parkClawArm()
        self.rfidarm_pubs['shoulder_pan_controller'].publish(0.00000000)
        self.rfidarm_pubs['elbow_tilt_controller'].publish(0.5522330520)
        self.rfidarm_pubs['wrist_tilt_controller'].publish(-1.764077805)
        self.__canUseRFID = True

    def scanAroundWithRFID(self):
        """
        Simulate a scanning motion with the RFID reader. This will only work if the RFID arm is
        positioned for use and the claw arm is parked.
        """
        if self.__canUseRFID:
            self.rfidarm_pubs['shoulder_pan_controller'].publish(-0.787443426)
            rospy.sleep(1)
            self.rfidarm_pubs['shoulder_pan_controller'].publish(0.7465372740)
            rospy.sleep(1)
            self.rfidarm_pubs['shoulder_pan_controller'].publish(0.0000000000)

    def moveClawToSide(self):
        """
        Raise the claw and position it to the left of the robot. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.raiseClaw()
            self.clawarm_pubs['shoulder_pan_controller'].publish(1.503301086)

    def openClaw(self):
        """
        Opens the claw to approximately 6" wide. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.clawarm_pubs['finger_left_controller'].publish(0.0)
            self.clawarm_pubs['finger_right_controller'].publish(0.0)

    def closeClaw(self):
        """
        Closes the claw such that the 1" wide phidgets accelerometer box can be moved. This will
        only work if the claw arm is positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.clawarm_pubs['finger_left_controller'].publish(-0.4550809410)
            self.clawarm_pubs['finger_right_controller'].publish(-0.383495175)

    def raiseClaw(self):
        """
        Raises the claw such that the object is approximately positioned above the robot's center of
        mass. This will only work if the claw arm is positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.clawarm_pubs['elbow_tilt_controller'].publish(0.1380582630)

    def putClawInFront(self):
        self.clawarm_pubs['shoulder_pan_controller'].publish(0.0)

    def lowerClaw(self):
        """
        Lowers the claw to the approximately 1.5" off the ground. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            self.clawarm_pubs['elbow_tilt_controller'].publish(-0.26077671899)
            self.putClawInFront()

    def rotateClaw(self):
        """
        Rotates the claw. This will only work if the claw arm is
        positioned for use and the RFID arm is parked.
        """
        if self.__canUseClaw:
            #print "rotated: " + str(self.__rotated)
            if self.__rotated:
                self.raiseClaw()
                self.clawarm_pubs['wrist_rotate_controller'].publish(0.0)
                self.__rotated = not self.__rotated
            else:
                self.raiseClaw()
                self.clawarm_pubs['wrist_rotate_controller'].publish(3.1190940899)
                self.__rotated = not self.__rotated

    def turnToRight(self):
        """
        Starts the robot wheels such that the robot spins clockwise.
        """
        self.drivertrain_pubs['right_front_wheel_controller'].publish(4)
        self.drivertrain_pubs['left_front_wheel_controller'].publish(4)
        self.drivertrain_pubs['right_rear_wheel_controller'].publish(4)
        self.drivertrain_pubs['left_rear_wheel_controller'].publish(4)

    def turnToLeft(self):
        """
        Starts the robot wheels such that the robot spins counter clockwise.
        """
        self.drivertrain_pubs['right_front_wheel_controller'].publish(-4)
        self.drivertrain_pubs['left_front_wheel_controller'].publish(-4)
        self.drivertrain_pubs['right_rear_wheel_controller'].publish(-4)
        self.drivertrain_pubs['left_rear_wheel_controller'].publish(-4)

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
    r = CharlieRobot()
    r.parkClawArm()
    r.parkRFIDArm()
    rospy.set_param('robot/validcommands', r.lookup.keys())
    rospy.set_param('robot/commandhelp', r.getCommandHelp())
    rospy.Subscriber('woz/robotcontrol', String, r.handleInput)
    print 'Robot is waiting for input.'
    rospy.spin()
