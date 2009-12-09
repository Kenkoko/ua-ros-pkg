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
roslib.load_manifest('ax12')

import sys
import rio
import time
import rospy
import threading
from Queue import Queue
from ax12.msg import Move
from ax12.msg import MotorData

class BioloidIO(threading.Thread):
    """
    All input to and output from the AX-12s goes through this class. This class is a Thread.
    While it is running, it reads from its input queue and writes the data to the serial port.
    If there are any subscribers to a dynamixel's topic (ie: "robot/ax12/dynamixel5") the motor's
    feedback data will be read from the serial port and published on the corresponding topic.
    Instances of a thread-safe data queue (Python's Queue) are used for all input to the run-loop.
    """
    
    def __init__(self, port, baud):
        """
        A ROS subscriber is set up to listen to the "commands" topic.
        Port and baudrate must be specified to reflect the hardware configuration.
        
            port - the path to the serial port to use for communicating with the robot
            
            baud - the baudrate at which the serial communication should occur
        """
        # initialize the Thread super class
        threading.Thread.__init__(self)
        
        # Set thread name
        self.setName('BioloidIOThread')
        
        # Get a monitor lock for thread safety
        self.__lock = threading.RLock()
        
        # Used in run loop
        self.__running = False
        
        # Hold info about the serial port
        self.__portName = port
        self.__baudrate = baud
        
        # This is the number of commands per sync-write that the run loop executes
        self.__MAX_CMDS_PER_WRITE = 4
        
        # IVAR for our AX12_IO object
        self.__axio = None
        
        # The input queue that messages from the ROS topic "bioloid/ax12/commands"
        # will be placed on. This is the queue that the run loop will process.
        self.__inQueue = Queue()
        
        # Get the available motor ids from the ROS parameter server.
        # The "main" waits for the init_sys.py to place this information
        # on the parameter server before starting this ROS node
        self.__motorIds = rospy.get_param('robot/ax12/motors')
        
        # Initialize the publishers for the available motors
        # Topics are named "robot/ax12/dynamixel" + motor_id
        # Topic publishes messages of type bioloid/MotorData
        self.__publishers = {}
        for id in self.__motorIds:
            self.__publishers[id] = rospy.Publisher('robot/ax12/dynamixel' + str(id), MotorData)

        # Add our handle_incoming_commands method as a subscriber to "bioloid/ax12/commands"
        rospy.Subscriber('robot/ax12/commands', Move, self.handle_incoming_commands)

    def stop(self):
        """
        Sets the running flag to false, thus breaking the run method's while loop.
        (Operation is thread safe)
        """
        self.setRunning(False)

    def setRunning(self, tOrF):
        """
        Thread safe setter for running boolean
        """
        self.__lock.acquire()
        self.__running = tOrF
        self.__lock.release()

    def isRunning(self):
        """
        Thread safe getter for running boolean
        """
        self.__lock.acquire()
        retVal = self.__running
        self.__lock.release()
        return retVal

    def __connect(self):
        """
        Connects the AX12_IO wrapper to the specified serial port for reading and writing.
        """
        self.__axio = rio.AX12_IO(self.__portName, self.__baudrate)

    def __disconnect(self):
        """
        Flushes the input and output buffers and closes the AX12_IO wrapper connection.
        """
        self.__axio.close()

    def handle_incoming_commands(self, move):
        """
        A simple handler for the commands that come in on the ROS topic.
        Places the Command object on the run loop's queue.
        """
        self.__inQueue.put(move)

    def __format(self, cmds):
        """
        Formats a list of Command objects into a tuple of command-tuples.
        Command-tuples take the form "(id, position, speed)".
        """
        cmdsList = []
        for cmd in cmds:
            cmdsList.append((cmd.id, cmd.position, cmd.speed))
        return tuple(cmdsList)

    def __publish_motor_data(self):
        for i in self.__motorIds:
            if self.__publishers[i].get_num_connections() > 0:
                dataDict = self.__axio.get_servo_feedback(i)
                if dataDict is not None:
                    motor_data = MotorData(dataDict['id'], dataDict['position'], dataDict['speed'], dataDict['load'], dataDict['voltage'], dataDict['temperature'])
                    self.__publishers[i].publish(motor_data)

    def __do_one_IO_loop(self):
        """
        The logic of the run loop. If the input queue is empty, just read motor state feedback. Otherwise send up to
        MAX_CMDS_PER_WRITE. After sending command(s), read the motor state feedback.
        """
        if self.__inQueue.empty():
            self.__publish_motor_data()
        else:
            cmds = [self.__inQueue.get_nowait()]
            numCmds = 1
            while numCmds < self.__MAX_CMDS_PER_WRITE and self.__inQueue.qsize() > 0:
                cmds.append(self.__inQueue.get_nowait())
                numCmds += 1
            cmdsTpl = self.__format(cmds)
            self.__axio.set_multi_servo_positions_and_speeds(cmdsTpl)
            time.sleep(0.005)
            self.__publish_motor_data()

    def run(self):
        """
        The run method that overrides the one defined in threading.Thread. This is called after calling
        bioloidIO_instance.start(). Calling this directly will not create a new Thread, the execution will
        occur on the caller's thread.
        """
        try:
            self.__connect()
        except rio.RIOException, e:
            print e.message, e.port, ' at ', e.baudrate

        self.setRunning(True)
        print 'Serial communication is running.\nUse ctrl + c to stop...'
        while self.isRunning():
            self.__do_one_IO_loop()
        print '\nStopping...'
        self.__disconnect()
        print self.getName() + ' Complete'
        return
    
if __name__ == '__main__':
    rospy.init_node('serialcomm', anonymous=True)
    while not rospy.has_param('robot/ax12/motors'):
        print 'System is not initialized, will try again in 3 seconds'
        time.sleep(3)
    
    # get port and baudrate from param server
    port = rospy.get_param('robot/ax12/port', '/dev/tty.usbserial-A9005MZc')
    baud = rospy.get_param('robot/ax12/baudrate', 1000000)
    
    robot_io = BioloidIO(port, baud)
    robot_io.start()
    while not rospy.is_shutdown():
        pass
    robot_io.stop()