#!/usr/bin/env python
import roslib; roslib.load_manifest('plotter')
import rospy

#from multiprocessing import Process
import matplotlib
matplotlib.use("cairo")
import matplotlib.pyplot as plt

import time

from plotter.msg import *
from plotter.srv import *

def plot(plot_data):
    plt.plot(plot_data.x_data, plot_data.y_data, label=plot_data.name)

    plt.xlabel(plot_data.x_label)
    plt.ylabel(plot_data.y_label)
    plt.title(plot_data.name)
    plt.legend(loc=0)
    
    plt.draw()
    
def handle_plot(req):
    t = time.time()
    plt.clf()
    for plot_data in req.plots:
        plot(plot_data)
    plt.savefig('../plots/' + str(t) + '.png')
    
    return PlotResponse()

def plot_server():
    rospy.init_node('plotter')
    s = rospy.Service('plot', Plot, handle_plot)
    plt.ion()
    print "Ready to plot things!", plt.isinteractive()
    
    rospy.spin()

if __name__ == "__main__":
    try:
        plot_server()
    except rospy.ROSInterruptException: pass