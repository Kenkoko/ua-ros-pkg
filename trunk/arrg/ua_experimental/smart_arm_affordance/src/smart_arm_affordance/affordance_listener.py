#!/usr/bin/env python
import roslib; roslib.load_manifest('smart_arm_affordance')
import rospy
import time
import math
import message_filters
# Matplotlib dependencies
import numpy as np
import pylab as pl
import cPickle
import matplotlib.pyplot as ax
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.path as path
import pprint
from pylab import *

# Commandline
import subprocess
# ROS imports
from std_msgs.msg import String
from ua_controller_msgs.msg import JointState
from phidgets_ros.msg import Float64Stamped

# Global variable to hold all of the sensor's data
sensorData = []
sensorDataAr = []
servoData = []
matrix = []
matrixAr = []

# Output file names
defaultDir = '/tmp/'
sensorOutputFile = defaultDir
servoOutputFile = defaultDir
plot6 = 'rxplot '
plot8 = 'rxplot '

# Variables depending on arm
#numSensors = 6
#sensorBoard = '/interface_kit/96952/sensor/'
#leftFinger = '/finger_left_controller/state'
#rightFinger = '/finger_right_controller/state'

numSensors = 8
sensorBoard = '/interface_kit/124427/sensor/'
leftFinger = '/left_finger_controller/state'
rightFinger = '/right_finger_controller/state'

# Make string to show sensor data on the fly
for i in range(numSensors):
    if(numSensors == 6):
        plot6 += sensorBoard + str(i) + '/data '
    elif(numSensors == 8):
        plot8 += sensorBoard + str(i) + '/data '
plot6 += ' > output.dat'
plot8 += ' > output.dat'

# Global variable to change Global sleep time between sensor reads
sleep_time = 0.02
# Global variable to show sensor output, 0 = no, 1 = yes
print_sensor_y_n = 0
print_servo_y_n = 0

# Start live sensor feed
print "Plotting..."
if(numSensors == 6):
    subprocess.Popen(plot6, shell=True)
elif(numSensors == 8):
    subprocess.Popen(plot8, shell=True)

#subprocess.Popen('./rxplot.sh', shell=True)

# Function to store sensor data depending on type, with option to print sensor's data
def store_sensor_data(datatype, index, msg, print_output):
    if datatype == 'sensorData':
        sensorData[index].append( (str(msg.header.stamp), msg.data) )
        if print_output == 1:
            print 'Sensor ' + str(index) + ': ' + str(msg.data) + "\n"
    elif datatype == 'servoData':
        servoData[index].append( (str(msg.header.stamp), msg.load) )
        if print_output == 1:
            if index == 0:
                print 'Left claw load: ' + str(msg.load) + '\n'
            else:
                print 'Right claw load: ' + str(msg.load) + '\n'
    time.sleep(sleep_time)

# Callback for sensor 0
def c0(msg):
    store_sensor_data('sensorData', 0, msg, print_sensor_y_n)
# Callback for sensor 1
def c1(msg):
    store_sensor_data('sensorData', 1, msg, print_sensor_y_n)
# Callback for sensor 2
def c2(msg):
    store_sensor_data('sensorData', 2, msg, print_sensor_y_n)
# Callback for sensor 3
def c3(msg):
    store_sensor_data('sensorData', 3, msg, print_sensor_y_n)
# Callback for sensor 4
def c4(msg):
    store_sensor_data('sensorData', 4, msg, print_sensor_y_n)
# Callback for sensor 5
def c5(msg):
    store_sensor_data('sensorData', 5, msg, print_sensor_y_n)
# Callback for sensor 6
def c6(msg):
    store_sensor_data('sensorData', 6, msg, print_sensor_y_n)
# Callback for sensor 7
def c7(msg):
    store_sensor_data('sensorData', 7, msg, print_sensor_y_n)
# Callback for Left finger
def callLeft(msg):
    store_sensor_data('servoData', 0, msg, print_servo_y_n)
# Callback for Right Finger
def callRight(msg):
    store_sensor_data('servoData', 1, msg, print_servo_y_n)

# Function to get all sensor data
def affordance_listener():
#    rospy.init_node('affordance_listener', anonymous=True)
    # Initialize the sensorData list of lists
    for i in range(numSensors):
        sensorData.append([])
    # Initialize the sensorData list of lists
    for i in range(2):
        servoData.append([])
    if(numSensors == 6):
        callbacks = (c0, c1, c2, c3, c4, c5)
    elif(numSensors == 8):
        callbacks = (c0, c1, c2, c3, c4, c5, c6, c7)
    # Get sensor data from each sensor, and call the associated callback
    for i in range(numSensors):
        interfaceKit = sensorBoard + '%d'
        rospy.Subscriber(interfaceKit % i, Float64Stamped, callbacks[i])
    # Get motor's load values
    rospy.Subscriber(leftFinger, JointState, callLeft)
    rospy.Subscriber(rightFinger, JointState, callRight)
    rospy.spin()

# Store the sensorData and servoData to an output file
def write_data():
    sensorOutput = open(sensorOutputFile, 'wb')
    servoOutput = open(servoOutputFile, 'wb')
    cPickle.dump(sensorData, sensorOutput)
    cPickle.dump(servoData, servoOutput)
    sensorOutput.close()
    servoOutput.close()

# Read the sensorData and servoData from an output file
def read_data(fileName):
    inputFile = open(fileName, 'rb')
    inputList = cPickle.load(inputFile)
    inputFile.close()
    return inputList

# Main
if __name__ == '__main__':
    rospy.init_node('affordance_listener', anonymous=True)
    sensorOutputFile += str(rospy.Time.now())
    sensorOutputFile += 'sensorData.dat'
    servoOutputFile += str(rospy.Time.now())
    servoOutputFile += 'servoData.dat'
    # Print output
    print 'Recording sensor data... (Press Ctrl-C to stop recording)'
    # Get input from sensors
    affordance_listener()
    # Store data to output file
    write_data()

