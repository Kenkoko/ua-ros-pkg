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

is_shown = False

def plot(plot_data):
    global is_shown
    t = time.time()
    
    plt.clf()
    plt.plot(plot_data.x_data, plot_data.y_data, label=plot_data.name)

    plt.xlabel(plot_data.x_label)
    plt.ylabel(plot_data.y_label)
    plt.title('plot' + str(t))
    plt.legend(loc=0)
    
    plt.draw()
    #if not is_shown:
    #    is_shown = True
    #    plt.show()
    plt.savefig('../plots/' + str(t) + '-plot.png')
 
# TODO: Need to make this multithreaded somehow   
def show():
    plt.show()
    
def handle_plot(req):
    for plot_data in req.plots:
        plot(plot_data)
        
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