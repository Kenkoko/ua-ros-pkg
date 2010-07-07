#!/usr/bin/env python
import roslib; roslib.load_manifest('smart_arm_affordance')
import rospy
import time
import math
import message_filters
#! Matplotlib dependencies
import numpy as np
import matplotlib.pyplot as ax
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.path as path
from pylab import *

# Commandline
import subprocess
# ROS imports
from std_msgs.msg import String
from ua_controller_msgs.msg import JointState
from phidgets_ros.msg import Float64Stamped
from numpy import *

# Global variable to hold all of the sensor's data
sensorData = []
servoData = []
histo = []
temphisto = []
matrix = []

# Global variable to change Global sleep time between sensor reads
sleep_time = 0.02
# Global variable to show sensor output, 0 = no, 1 = yes
print_sensor_y_n = 0
print_servo_y_n = 1

# Start live sensor feed
subprocess.Popen('./rxplot.sh', shell=True)

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
def callLeft(msg):
    store_sensor_data('servoData', 0, msg, print_servo_y_n)
def callRight(msg):
    store_sensor_data('servoData', 1, msg, print_servo_y_n)

# Function to get all sensor data
def affordance_listener():
    rospy.init_node('affordance_listener', anonymous=True)
    # Initialize the sensorData list of lists
    for i in range(6):
        sensorData.append([])
    # Initialize the sensorData list of lists
    for i in range(2):
        servoData.append([])
    callbacks = (c0, c1, c2, c3, c4, c5)
    # Get sensor data from each sensor, and call the associated callback
    for i in range(6):
        rospy.Subscriber("/interface_kit/96952/sensor/%d" % i, Float64Stamped, callbacks[i])
    # Get motor's load values
    rospy.Subscriber("/finger_left_controller/state", JointState, callLeft)
    rospy.Subscriber("/finger_right_controller/state", JointState, callRight)
    rospy.spin()

# Plot the sensor's correspinding color and data point
def plot_sensor_data(inputList, color):
    for d in inputList:
        if color == 0:
            ax.plot(str(d[0]), str(d[1]),'ro')
        elif color == 1:
            ax.plot(str(d[0]), str(d[1]),'g^')
        elif color == 2:
            ax.plot(str(d[0]), str(d[1]),'bo')
        elif color == 3:
            ax.plot(str(d[0]), str(d[1]),'c^')
        elif color == 4:
            ax.plot(str(d[0]), str(d[1]),'mo')
        elif color == 5:
            ax.plot(str(d[0]), str(d[1]),'k^')

# Plot the sensor data over time
def graph_data_scatter():
    ax.title('Sensor Object Clustering')
    ax.xlabel('Time')
    ax.ylabel('Sensor value')
    # Graph the output
    print
    for index, i in enumerate(sensorData):
        print 'Plotting sensor ' + str(index)
        plot_sensor_data(i, index)
    ax.show()

# Helper for the histogram plotter
def plot_subhisto(sensor, sensorType, data, numBins):
    # Histogram code (matplotlib)
    x =     arange(numBins)
    temp = []
    # Make a list sublist sizes
    for i in data:
            temp.append(float(len(i)))
    sum_histo = 0
    # Calculate the sum of all values
    for j in temp:
        sum_histo += j
    # Normalize values with sum of all values
    for k in range(len(temp)):
        temp[k] = float(temp[k] / sum_histo)
    # Title type in subplot
    if sensorType == 0:
        title = 'Sensor ' + str(sensor)
    else:
        title = 'Servo ' + str(sensor - 6)
    # Histogram code (matplotlib)
    ax.title(title)
    ax.ylabel('Frequency')
    ax.xlabel('Bins')
    bar(x, temp)
    xticks( x + 0.5,  range(numBins) )

# Plot histogram for all 6 sensors and 2 servos
def graph_data_histo(matrixData, numBins):
    subplot(2,4, 1)
    plot_subhisto(0, 0, matrixData[0], numBins)
    subplot(2,4, 2)
    plot_subhisto(1, 0, matrixData[1], numBins)
    subplot(2,4, 3)
    plot_subhisto(2, 0, matrixData[2], numBins)
    subplot(2,4, 4)
    plot_subhisto(3, 0, matrixData[3], numBins)
    subplot(2,4, 5)
    plot_subhisto(4, 0, matrixData[4], numBins)
    subplot(2,4, 6)
    plot_subhisto(5, 0, matrixData[5], numBins)
    subplot(2,4, 7)
    plot_subhisto(6, 1, matrixData[6], numBins)
    subplot(2,4, 8)
    plot_subhisto(7, 1, matrixData[7], numBins)
    show()


def convert_to_numpy(inputList, index):
    temp = []
    for i in inputList[index]:
        temp.append(i[2])
        
    num = array(temp)
    num.sort()
    return num

def toList(inputList, index):
    temp = []
    for i in inputList[index]:
        temp.append(i[1])
    return temp

# Insert the value into the proper place, also error check values
def insert_into_bucket(data, value, minVal, maxVal, bucketWidth, Nbuckets):
    internalVal = float(value)
    # if the insert value is smaller than the minimum value
    if internalVal < minVal:
        print
        print 'ERROR: ' + str(value) + ' is out of range for min of ' + str(minVal)
        print
    # if the insert value is larger than the max value
    elif internalVal > maxVal:
        print
        print 'ERROR: ' + str(value) + ' is out of range for max of ' + str(minVal)
        print
    elif maxVal == 0:
        print 'No data to add to buckets.  List was all zero...'
        return 1
    # insert that value
    else:
        index = int(internalVal/bucketWidth)
        data[index].append(internalVal)
    return 0

# Convert the data into buckets for a histogram
def bucket_counter(data, minV, maxV, Nbuckets):
    minVal = float(minV)
    maxVal = float(maxV)
    rangeVal = maxVal - minVal
    temphisto = []
#    print
#    print 'Value range: ' + str(rangeVal)
    bwidth = rangeVal / Nbuckets
#    print 'Bucket width: ' + str(bwidth)
#    print
    # Create a list with N buckets
    for i in range(Nbuckets):
        temphisto.append([])
    # Insert the sensor data into the buckets
    for j in data:
        result = insert_into_bucket(temphisto, j, minVal, maxVal, bwidth, Nbuckets)
        if result == 1:
            temphisto[0] = data
            break
    # Give back the whole list
    return temphisto

# Main
if __name__ == '__main__':
    # Print output
    print 'Getting and recording sensor data... (Press Ctrl-C to stop recording)'
    # Get input from sensors
    affordance_listener()
    # Take all sensor data, put them into buckets, then append to the output matrix
    for i in range(6):
        temp = toList(sensorData, i)
        temphisto = bucket_counter(temp, min(temp), max(temp) + 1, 10)
        matrix.append(temphisto)
    # Take all servo data, put them into buckets, then append to the output matrix
    for j in range(2):
        temp = toList(servoData, j)
        temphisto = bucket_counter(temp, min(temp), max(temp) + 1, 10)
        matrix.append(temphisto)
    # Graph the output #
#    graph_data_scatter()
    graph_data_histo(matrix, 10)

