#!/usr/bin/env python

# matplotlib
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import rc

# numpy
import numpy as np

import os
import time
import csv
import sys
#from operator import itemgetter, attrgetter

def confidence(p, n, z):
    n = float(n)
    num = p + (1 / (2*n)) * (z*z) + z * np.sqrt( ((p * (1-p)) / n) + (z*z)/(4*n*n) )
    denom = 1 + (1 / n) * (z*z)
    return (num / denom) - p

def stdErr(p, n):
    std_dev = np.sqrt( (p * (1-p)) / n)
    return std_dev / np.sqrt(n)

def get_data(verb, condition):
    folder = verb + '_' + condition

    if not os.path.exists(folder):
        return None

    success = []
    times = []
    f_scores = []

    row_count = 0
    row_count_2 = 0
    for file_name in os.listdir(folder):
        if file_name.startswith('.'):
            continue

        if file_name.startswith('r'):
            file_reader = csv.reader(open(folder + "/" + file_name))
            for row in file_reader:
                if (row_count_2 + 1) % 3 == 0:
                    f_scores += [row]
                row_count_2 += 1
        else:
            file_reader = csv.reader(open(folder + "/" + file_name))
            for row in file_reader:
                if row_count % 2 == 0:
                    success += [row]
                else:
                    times += [row]
                row_count += 1

    # Process the execution results
    n = len(success)

    if n == 0:
        return None

    success_count = [0.0 for i in range(0, len(success[0]))]

    for row in success:
        for i in range(0, len(row)):
            if row[i] == 'Success':
                success_count[i] += 1

    means = np.divide(success_count, n) 
    intervals = [0.0 for i in range(0, len(means))]    
    for i in range(0, len(means)):
        intervals[i] = stdErr(means[i], n)

    # Process the recognition results
    n_r = len(f_scores)
    means_r = []
    err_r = []

    if len(f_scores) > 0:
        for col in range(0, len(f_scores[0])):
            values = []
            for row in range(0, n_r):
                values += [float(f_scores[row][col])]
            mean = np.mean(values)
            std = np.std(values)
            err = std / np.sqrt(n_r)
            means_r += [mean]
            err_r += [err]

    #print means_r

    return [means, intervals, n, means_r, err_r, n_r]

def make_execution_plot(ax, verb, curves, colors, is_first):
    clean_verb = verb.replace('_', '-')
    #ax.set_title(r'Probability of Student Success for {\it ' + clean_verb + '}')
    ax.set_title(r'{\it ' + clean_verb + '}')

    plot_x = []

    for condition, plot_data in curves.items():
        plot_y = plot_data[0]
        plot_err = plot_data[1]
        n = plot_data[2]
        plot_x = range(1, len(plot_y)+1)

        print "exe: ", verb, condition, n
        label = condition[0] #+ ' (n=' + str(n) + ')'
        ax.errorbar(plot_x, plot_y, yerr=plot_err, label=label, color=colors[condition]) #, ls='-', color='black')

    # Axis dimensions     
    ax.axis([1,len(plot_x)+0.1,0.0,1.0])
    # Axis Ticks
    ax.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    ax.get_yticklabels()[0].set_visible(False)
    #ax.set_xticks(plot_x)
    # Add a grid
    ax.grid(color='0.6')
    # Remove spines and ticks from top and right axes
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    plt.xlabel("Teaching Episode")

    # Hide the labels on the x axis
    plt.setp(ax.get_xticklabels(), visible=False)
    # Shrink the font on the y axis
    plt.setp(ax.get_yticklabels(), fontsize='9.0')

    if not is_first:
        plt.setp(ax.get_yticklabels(), visible=False)
        #ax.get_xticklabels().setVisible(false)

    if is_first:
        ax.legend(loc=4)
        #plt.ylabel('Probability of Success')

    #plt.ylabel('Probability of Student Success')
    #ax.legend(loc=4)

def make_recognition_plot(ax, verb, curves, colors, is_first):
    clean_verb = verb.replace('_', '-')
    #ax.set_title(r'Recognition F-score for {\it ' + clean_verb + '}')

    plot_x = []

    for condition, plot_data in curves.items():
        plot_y = plot_data[3]
        plot_err = plot_data[4]
        n = plot_data[5]
     
        if n > 0:
            print "rec: ", verb, condition, n
            plot_x = range(1, len(plot_y)+1)
            label = condition# + ' (n=' + str(n) + ')'
            ax.errorbar(plot_x, plot_y, yerr=plot_err, label=label, color=colors[condition]) #, ls='-', color='black')

    # Axis dimensions     
    ax.axis([1,len(plot_x)+0.1,0.0,1.0])
    # Axis Ticks
    ax.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
     # Shrink the font on the y axis
    plt.setp(ax.get_yticklabels(), fontsize='9.0')
    # Add a grid
    ax.grid(color='0.6')
    # Remove spines and ticks from top and right axes
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    plt.xlabel("Teaching Episode")

    if is_first:
        plt.ylabel('Recognition F-score')
    else:
        plt.setp(ax.get_yticklabels(), visible=False)
    

def plot_verb(verb, conditions, ex, rec, is_first):
    color_list = ['b', 'g', 'r', 'k']

    curves = {}
    colors = {}

    color_index = 0
    for condition in conditions:
        data = get_data(verb, condition)
        if data:
            curves[condition] = data
        colors[condition] = color_list[color_index]
        color_index += 1

    # Make the execution graph
    make_execution_plot(ex, verb, curves, colors, is_first)

    # Make the recognition graph
    make_recognition_plot(rec, verb, curves, colors, is_first)




if __name__ == "__main__":
    plt.ion()

    verb = sys.argv[1] 
    conditions = sys.argv[2:]

    verbs = [verb]
    if verb == 'all':
        verbs = ['go', 'go_via', 'follow', 'intercept', 'avoid']

    #print verbs

    # Use LaTeX for font rendering
    rc('text', usetex=True)
    rc('font', family='serif', serif='Times', size='10.0')
    rc('legend', fontsize=9)

    fig = plt.figure(1, (10.0, 4.0), frameon=False)
    fig.subplots_adjust(hspace=0.05)
    fig.subplots_adjust(wspace=0.05)
    #fig = plt.figure(1, figsize=(4.0,2.5))

    for i, v in zip(range(len(verbs)),verbs):
        ex = fig.add_subplot(2, len(verbs), i+1)
        if i == 0:
            plt.ylabel('Probability of Success')
        rec = fig.add_subplot(2, len(verbs), i+len(verbs)+1)
        plot_verb(v, conditions, ex, rec, (i == 0))    

   

    #fig.set_size_inches(4.0,2.5)
    fig.savefig(verb + '.pdf', bbox_inches="tight")

    # Show it
    #plt.show()

    exit()
