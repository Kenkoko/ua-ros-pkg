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
import pprint, pickle
from pylab import *

# Commandline
import subprocess
from optparse import OptionParser
# ROS imports
from std_msgs.msg import String
from ua_controller_msgs.msg import JointState
from phidgets_ros.msg import Float64Stamped

# Global variable to hold all of the sensor's data
sensorData = []
sensorDataAr = []
servoData = []
histo = []
temphisto = []
matrix = []
matrixAr = []

# Variables depending on arm
numSensors = 6

# Output file names
sensorOutputFile = 'sensorData.dat'
servoOutputFile = 'servoData.dat'

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
        elif color == 6:
            ax.plot(str(d[0]), str(d[1]),'b^')
        elif color == 7:
            ax.plot(str(d[0]), str(d[1]),'r^')

# Plot the sensor data over time
def graph_data_scatter(inputList):
    ax.title('Sensor Object Clustering')
    ax.xlabel('Time')
    ax.ylabel('Sensor value')
    # Graph the output
    for index, i in enumerate(inputList):
        print 'Plotting sensor ' + str(index)        
        plot_sensor_data(i, index)
    ax.show()

# Helper for the histogram plotter
def plot_subhisto(sensor, sensorType, data, numBins):
    # Histogram code (matplotlib)
    x = arange(numBins)
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
    bwidth = rangeVal / Nbuckets
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

def convert_to_numpy(inputList, index):
    temp = []
    for i in inputList[index]:
        temp.append(i[1])
    num = array(temp)
    return num

def toList(inputList, index):
    temp = []
    for i in inputList[index]:
        temp.append(i[1])
    return temp

def histo_generator():
    # Take all sensor data, put them into buckets, then append to the output matrix
    for i in range(numSensors):
        temp = toList(sensorData, i)
        temphisto = bucket_counter(temp, min(temp), max(temp) + 1, 10)
        matrix.append(temphisto)
    # Take all servo data, put them into buckets, then append to the output matrix
    for j in range(2):
        temp = toList(servoData, j)
        temphisto = bucket_counter(temp, min(temp), max(temp) + 1, 10)
        matrix.append(temphisto)

# Store the sensorData and servoData to an output file
def write_data():
    sensorOutput = open(sensorOutputFile, 'wb')
    servoOutput = open(servoOutputFile, 'wb')
    pickle.dump(sensorData, sensorOutput)
    pickle.dump(servoData, servoOutput)
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
    graph_type = ''
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="filename", help="write report to FILE", metavar="FILE")
    (options, args) = parser.parse_args()
    if(len(args) > 0):
        sensorOutputFile = args[0]
        servoOutputFile = args[1]
        if(len(args) > 2):
            graph_type = args[2]
    else:
        print 'No input files specified!  Opened from current directory...'

    # Read in data from output file
    sensorData = read_data(sensorOutputFile)
    servoData = read_data(servoOutputFile)
    print sensorData
    if(graph_type == 'histo'):
        histo_generator()
        graph_data_histo(matrix, 10)
    elif(graph_type == 'scatter'):
        # Graph the output #
        graph_data_scatter(sensorData)
    else:
        print 'Please specify graph type: histo or scatter'





